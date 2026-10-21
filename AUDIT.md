# Audit zpráva — rp-pico_mqtt

---

## Audit #8 — dlouhodobá stabilita (verze 1.5.2)

**Datum:** 2026-10-20
**Soubory:** `main.py`, `postgres-push/mqtt_to_postgres.py`, `postgres-push/mqtt-to-postgres.service`
**Zaměření:** chyby, které se projeví až po dnech nebo týdnech provozu: přetečení `ticks_ms`,
stavy, ze kterých se zařízení samo nedostane, a skryté resety WDT.
**Ověření:** přetečení `ticks` simulací (snippet u L1), chování `umqtt.simple` proti upstream
zdroji (micropython-lib, `umqtt.simple` 1.8.1), regrese NTP proti `git show 2955047`.

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| L1 | ~~KRITICKÁ~~ | Zastaralý backoff deadline + přetečení `ticks_ms` → reconnect MQTT i ADC zablokovaný až na 6,2 dne | **OPRAVENO** (v1.5.3) |
| L2 | ~~VYSOKÁ~~ | Regrese B1 (#3) ve v1.4.0: když je NTP nedostupné, zkouší se v **každé** smyčce (5 s blok); dvě volání v jedné iteraci = WDT reset | **OPRAVENO** (v1.5.3) |
| L3 | NÍZKÁ (původně VYSOKÁ) | `umqtt` `wait_msg()` volá `setblocking(True)` → po prvním pingu je `settimeout(2)` pryč; zombie spojení končí WDT resetem, ne reconnectem | Otevřeno, vědomě neopraveno (přehodnoceno 2026-10-21) |
| L4 | STŘEDNÍ | `_mqtt_auth_err` navždy (= D1 z #6): jedno přechodné rc 4/5 od brokeru vypne MQTT až do ručního resetu | Otevřeno |
| L5 | ~~STŘEDNÍ~~ | Před prvním úspěšným NTP se publikují časy z roku 2021 a démon je zapíše do DB | **OPRAVENO** (v1.5.3) |
| L6 | NÍZKÁ | Únik socketu v chybové větvi `connect_mqtt()` | Otevřeno |
| L7 | NÍZKÁ | Chybí eskalace: zaseknutý CYW43 nebo I2C sběrnice se neobnoví bez restartu napájení | Otevřeno, řešit jen při výskytu |
| L8 | ~~NÍZKÁ~~ | Tlačítko trpí stejnou chybou jako L1: po více než 6,2 dne bez stisku může až 6,2 dne nereagovat | **OPRAVENO** (v1.5.3) |
| L9 | INFO | Skryté resety WDT nejsou vidět: diag nemá `reset_cause` a démon diag zahazuje (S4) | Otevřeno |

Nálezy k démonovi (B8–B10) jsou v `postgres-push/AUDIT.md`.

---

### L1 — Zastaralý deadline + přetečení `ticks_ms` (KRITICKÁ) — OPRAVENO ve v1.5.3

**Soubor:** `main.py:159`, `main.py:171`, `main.py:241`

`ticks_ms()` na rp2 má periodu 2^30 ms (12,4 dne) a `ticks_diff()` vrací znaménkový rozdíl
v rozsahu ±2^29 ms (±6,2 dne). Deadline starší než 6,2 dne proto vypadá jako budoucí.

- `check_wifi()` po reconnectu nastaví `_mqtt_next_try = 0`. Pokud je uptime v druhé
  polovině periody (6,2–12,4 d, 18,6–24,8 d, …), `connect_mqtt()` vrací `False` na ř. 241
  až do příštího přetečení: **až 6,2 dne bez MQTT**, na displeji E--2, a v bufferu zůstane
  jen posledních 200 zpráv.
- Totéž bez výpadku Wi-Fi: `_mqtt_next_try` zůstává z posledního selhání. Po 7 dnech
  stabilního spojení stačí jeden zombie-close a reconnect je zablokovaný.
- ADC: `_adc_next_try` zůstává 0 (nebo hodnota z posledního selhání). Jakýkoli I2C glitch po
  6,2 dne provozu → `_init_adc()` se vrátí hned na ř. 171 → **E--3 až 6,2 dne**.

Ověřeno simulací:

```python
PERIOD, HALF = 1 << 30, 1 << 29
def ticks_diff(a, b): return ((a - b + HALF) % PERIOD) - HALF
assert ticks_diff(int(6.3 * 86_400_000), 0) < 0   # uptime 6,3 d: "deadline 0 je v budoucnu"
```

**Oprava (3 řádky):** při ztrátě spojení posunout deadline na „teď“, aby nikdy nebyl starý:

```python
# check_wifi(), ř. 159
self._mqtt_next_try = time.ticks_ms()
# _close_mqtt(), uvnitř `if self.mqtt is not None:`
self._mqtt_next_try = time.ticks_ms()
# read_adc_channel(), except OSError vedle `self.adc = None`
self._adc_next_try = time.ticks_ms()
```

Za běžných podmínek se chování nemění, protože deadline už je v minulosti a první pokus
proběhne hned. Ve všech ostatních místech (`_wifi_deadline`, `_mqtt_last_ping`) se deadline
vždy nastavuje nedávno, ty jsou v pořádku.

**Implementováno ve v1.5.3 jinak** (po code review): místo resetů na třech místech se
v `connect_mqtt()` a `_init_adc()` čeká jen na deadline v okně backoffu,
`-backoff <= ticks_diff(now, next_try) < 0`. Pokrývá i cesty bez resetu, např. když Wi-Fi
naskočí mimo reconnect cyklus. Hlídá to `selfcheck.py`.

---

### L2 — NTP retry v každé smyčce (VYSOKÁ, regrese B1) — OPRAVENO ve v1.5.3

**Soubor:** `main.py:635–640`, `main.py:160`

Verze 1.2.4 opravila B1 tak, že při selhání NTP posunula `_offset_day` na dnešek. V1.4.0
(`2955047`) při odstraňování DST tuto větev smazala:

```python
if t[3] >= 1 and t[2] != self.shared._last_ntp_day:
    if self.shared.wlan.isconnected():
        self.shared._sync_ntp()          # při selhání se _last_ntp_day nezmění
```

Scénář „LAN funguje, internet ne“ (výpadek WAN, restart routeru): od 01:00 UTC se NTP zkouší
v každé iteraci. Každý pokus = DNS (`pool.ntp.org`) + 5 s timeout, takže měření běží místo
každou 1 s každých ~6 s, a to po celý den. Pokud se ve stejné iteraci dokončí Wi-Fi reconnect,
`check_wifi()` volá `_sync_ntp()` podruhé, bez krmení WDT mezi tím: 2 × 5 s > 8 s → **WDT reset**,
a po něm hned L5.

**Oprava:**

```python
if t[3] >= 1 and t[2] != self.shared._last_ntp_day:
    self.shared._last_ntp_day = t[2]     # jeden pokus za den, i při selhání
    if self.shared.wlan.isconnected():
        self.shared._sync_ntp()
```

a na začátek `_sync_ntp()` přidat `if self.wdt: self.wdt.feed()`. To zároveň zmírní N6.

---

### L3 — `umqtt` ruší socket timeout po prvním pingu (NÍZKÁ, původně VYSOKÁ)

**Soubor:** `main.py:287`, `main.py:324–325`

Upstream `umqtt.simple`:

```python
def check_msg(self):
    self.sock.setblocking(False)
    return self.wait_msg()

def wait_msg(self):
    res = self.sock.read(1)
    self.sock.setblocking(True)      # ← timeout = None, settimeout(2) je pryč
    if res is None:
        return None
```

Důsledky:

1. Po prvním `mqtt_ping()` (30 s po connectu) je socket plně blokující. `publish()`
   nebo `ping()` na mrtvém spojení po zaplnění TCP send bufferu visí, dokud nepřijde **WDT
   reset**. Zařízení se tedy restartuje místo reconnectu a přijde o buffer.
2. `check_msg()` je neblokující a volá se hned po `ping()`, takže PINGRESP aktuálního pingu
   ještě nedorazil. Přečte se PINGRESP z pingu o 30 s dříve. Na zombie socketu, kam nic
   nepřichází, vrátí `None`, takže **detekce zombie popsaná v `implementation_notes.md` §3
   timeoutem nefunguje**. Odpojení se pozná jen podle RST/FIN, nebo až díky WDT.

**Oprava (otestovat na zařízení alespoň 24 h, mění empiricky laděné chování z §1):**

```python
self.mqtt.ping()
self.mqtt.wait_msg()            # čeká max 2 s na PINGRESP, jinak OSError → _close_mqtt()
self.mqtt.sock.settimeout(2)    # wait_msg() volá setblocking(True) a timeout ruší
```

Nejhorší případ bez krmení WDT jsou 2 × 2 s. **Ověřit:** kterou verzi `umqtt.simple` má
firmware (frozen modul). `setblocking(True)` je ve `wait_msg()` dlouhodobě, ne jen v 1.8.1.

**Přehodnocení (2026-10-21) — snížено na NÍZKOU, vědomě neopraveno:**

- **Kdy nastane:** jen při zombie spojení (WiFi „připojená", k brokeru nedojde nic, ani RST),
  které trvá déle než zaplnění TCP send bufferu. Při 2 senzorech a `INTERVAL_S = 1`
  (~160 B/s) a `TCP_SND_BUF` řádově 11 kB (8 × MSS, neověřeno ve firmwaru) je to ~1 min.
  Restart procesu brokeru pošle RST a L3 nevyvolá; krátký deauth/roaming AP (§3 naměřeno
  74 s) je na hraně. Reálný scénář: výpadek serveru s brokerem nebo sítě mezi AP a
  brokerem > 1 min. Více senzorů nebo kratší interval okno úměrně zkracuje.
- **Dopad:** WDT reset a čistý boot. Data ze zombie období se ztratí i bez L3 (leží v TCP
  bufferu), `_pub_buffer` je v tu chvíli prázdný. Navíc se ztratí jen hodnoty po bootu do
  první NTP synchronizace, pokud výpadek sítě znemožní i NTP (L5: před NTP se nebufferuje).
- **Proč neopravovat:** oprava výše mění empiricky laděné chování CYW43 (§1, §3), ladění
  stabilního spojení s AP bylo pracné. V provozu se L3 zatím neprojevil: `uptime` v
  `diag/*` po nasazení v1.5.3 (MS1 ráno, L200h ~12:02 UTC) bez neplánovaného restartu.
- **Sledování:** pokles `uptime` v retained `diag/<box>` bez nasazení/odpojení = neplánovaný
  restart. `machine.reset_cause()` by WDT od L3 neodlišil od `E--5` (`machine.reset()` jde
  na rp2 přes watchdog), jen od výpadku napájení.
- **Záložní oprava při výskytu (varianta A)** — vzor čtení PINGRESP zůstává beze změny,
  socket se vrací do stavu, ve kterém je už dnes prvních ~30 s po každém connectu:

  ```python
  self.mqtt.ping()
  self.mqtt.check_msg()           # beze změny
  self.mqtt.sock.settimeout(2)    # check_msg() timeout ruší, obnovit
  ```

  Řeší důsledek 1 (visící `publish()` → `OSError` → `_close_mqtt()` místo WDT resetu),
  důsledek 2 ne. Riziko: zápis zdržený > 2 s na zdravém spojení vyvolá reconnect.

---

### L4 — `_mqtt_auth_err` navždy (STŘEDNÍ, povýšení D1)

**Soubor:** `main.py:235–236`, `main.py:299–302`

Pro dlouhodobý běh je to víc než designová drobnost: když broker jednou vrátí rc 5 (reload
s rozbitým ACL nebo password souborem, chvíle při startu pluginu), zařízení už nikdy nic
nepublikuje, i když broker vzápětí funguje. Totéž dělá démon (B9).

**Oprava:** místo trvalého flagu použít dlouhý backoff:

```python
# smazat `if self._mqtt_auth_err: return False`
if isinstance(e, MQTTException) and e.args and e.args[0] in (4, 5):
    print("MQTT auth failed:", e)
    self._mqtt_auth_err = True                           # displej dál ukazuje E--6
    self._mqtt_next_try = time.ticks_add(now, 300_000)   # zkusit znovu za 5 min
    return False
# a při úspěšném connectu:
self._mqtt_auth_err = False
```

---

### L5 — Timestampy z roku 2021 před prvním NTP (STŘEDNÍ) — OPRAVENO ve v1.5.3

**Soubor:** `main.py:416–419`

Po výpadku proudu naběhne Pico s RTC 2021-01-01. Pokud první NTP selže (Wi-Fi nebo WAN
naběhne později než Pico), nejbližší další pokus přijde při Wi-Fi reconnectu, nebo až v
RTC „den 2, 01:00“, tedy **~25 h dat s rokem 2021** v DB. Démon přitom umí payload bez
času orazítkovat časem příjmu (`mqtt_to_postgres.py:130`).

**Oprava:**

```python
t = time.gmtime()
# ponytail: před NTP bez času, démon orazítkuje příjmem; u bufferovaných zpráv to znamená chybu až o délku bufferu
ts = " {:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(*t[:6]) if t[0] >= 2025 else ""
formatted = "{:.{}f}{}".format(self.last_value, self.precision, ts)
```

S opravou L2 (pokus o NTP jednou denně) je tohle nutné. Jinak by se po neúspěšném NTP
při bootu čekalo celý den s nesmyslným časem.

---

### L6 — Únik socketu při neúspěšném connectu (NÍZKÁ)

**Soubor:** `main.py:293–297`

`client.disconnect()` nejdřív zapisuje a teprve potom zavírá. Když zápis selže, socket zůstane
otevřený. Pravděpodobně ho zavře finalizer při GC, ale to na zařízení ověřené není.
`_close_mqtt()` má pro stejný případ záložní `sock.close()`, tady chybí. Oprava: na místě
`client.disconnect()` zavřít rovnou `client.sock.close()`, DISCONNECT paket při selhaném
connectu nemá smysl.

---

### L7 — Chybí eskalace pro zaseknutý hardware (NÍZKÁ)

- **CYW43:** `start_wifi_reconnect()` opakuje `wlan.connect()` donekonečna. Pokud se čip
  zasekne (u Pico W známá třída problémů), pomůže jen restart. Levná pojistka je počítat
  vypršené `_wifi_deadline` a po ~10 (15 min) zavolat `machine.reset()`. Ztráta bufferu
  nevadí, po 15 min výpadku už stejně drží jen posledních 200 zpráv.
- **I2C:** když ADS1115 drží SDA v nule (typicky po resetu masteru uprostřed transakce),
  `i2c.scan()` vrací `[]` donekonečna a `machine.reset()` nepomůže, protože ADS1115 se
  neresetuje. Řešením je bus recovery (9 pulzů SCL přes GPIO a znovu vytvořit `I2C`)
  v `_init_adc()`.

Obojí řešit **jen pokud se trvalé E--1 / E--3 v provozu opravdu objeví**.

---

### L8 — Tlačítko po dlouhém klidu nereaguje (NÍZKÁ) — OPRAVENO ve v1.5.3

**Soubor:** `main.py:578`

`ticks_diff(now, self._btn_last) < 200` vyjde záporně, pokud je poslední stisk (nebo 0 po
bootu) starší než 6,2 dne. Funkce se pak vrátí ještě před aktualizací `_btn_last`, takže
tlačítko zůstane mrtvé až do přetečení. Oprava: `if 0 <= time.ticks_diff(now, self._btn_last) < 200:`.

---

### L9 — Skryté resety nejsou vidět (INFO)

L2 i L3 končí WDT resetem, který z venku vypadá jen jako krátký výpadek. Jedinou stopou je
pokles `uptime` v diag, jenže démon diag zahazuje (S4) a retained zpráva drží jen poslední
hodnotu. Levné zlepšení: při bootu uložit `machine.reset_cause()` a posílat ho v diag jako
`"rst"` (rozliší WDT_RESET od PWRON_RESET). Historii pak stačí sbírat
`mosquitto_sub -t 'diag/#' -v` do souboru.

---

### Ověřeno OK z hlediska dlouhodobého běhu

| Oblast | Výsledek |
|--------|----------|
| Zápisy do flash | žádné, opotřebení nehrozí ✓ |
| `_uptime_s` | Python int bez přetečení ✓ (drift D3 trvá) |
| `_wifi_deadline`, `_mqtt_last_ping` | nastavují se vždy čerstvě, ticks wrap jim nevadí ✓ |
| `_pub_buffer` | omezený na `PUBLISH_BUFFER_MAX`, paměť nemůže růst ✓ |
| Nezachycená výjimka v `run()` | FATAL → `reset()` po 5 s, nikdy trvalý stav ✓ |
| Nekonečná smyčka v `ads1x15.read()` (polling OS bitu bez timeoutu) | pokryto WDT ✓ |
| Pin IRQ → `micropython.schedule` | výjimka v naplánovaném callbacku hlavní smyčku neshodí ✓ |
| `ticks_ms() % 2500` v `_update_display` | při přetečení jen jednorázový glitch blikání ✓ |

---

## Audit #7 — externí review (verze 1.5.1)

**Datum:** 2026-08-17
**Soubory:** `main.py`, `config.py`
**Zdroj:** externí model (`quen_audit.txt`, 14 nálezů), ověřeno proti kódu, git historii a `knihovny/`

Ze 14 nálezů tři platily, jeden audit minul, deset zamítnuto. Zamítnutí jsou tu zapsaná
se zdůvodněním, aby se nálezy nevracely v dalších kolech.

| ID | Nález | Verdikt |
|----|-------|---------|
| Q1 | `MQTTClient.connect()` bez timeoutu | **PLATÍ** = A3 → **OPRAVENO** (v1.5.2). Navržená oprava (monkey-patch `client.sock`) byla chybná, `connect()` si socket přepisuje. |
| Q2 | `create_sensor()` chytá jen `(KeyError, ValueError)` — `TypeError` propadne | **PLATÍ** → **OPRAVENO** (v1.5.2), `except Exception`. |
| Q3 | `sum(vals) // len(vals)` zahazuje až 0,5 LSB | **PLATÍ** → **OPRAVENO** (v1.5.2), float průměr; rozdíl 0,004 kPa na rozsahu 0–120 kPa. |
| Q4 | `E--5` bez restartu — WDT se zapíná až v `run()`, výjimka z `__init__` nechá zařízení stát navěky | **NOVÉ** (audit to minul, popsal jen nezachycený `I2C()` OSError) → **OPRAVENO** (v1.5.2), `reset()` po 5 s. |
| Q5 | Hesla v `config.py` = „bezpečnostní katastrofa" | **ZAMÍTNUTO** — `config.py` je v `.gitignore` od `ba57aff`, v historii jen placeholdery. Reálná expozice byl `cat config.py \| llm`, kterým audit vznikl, ne git. |
| Q6 | TM1637 neumí záporné znaménko, `-5.5` se zobrazí jako `+5.50` | **ZAMÍTNUTO** — `tm1637.py` mapuje `'-'` (ord 45) na `_SEGMENTS[37]` = `0x40`. `"-5.50"` = 4 pozice, zobrazí se správně. Navržená oprava by funkční věc rozbila. |
| Q7 | `if i % 10 == 9` — WDT se ve vzorkovací smyčce nekrmí vůbec | **ZAMÍTNUTO** — pojistka pro vysoké `ADC_SAMPLES`, tam funguje. Audit počítal špatně: 200 vzorků při 64 SPS = 3,1 s, ne >8 s. Viz `PONYTAIL_AUDIT.md` #12. |
| Q8 | Odstranit TCP pre-test | **ZAMÍTNUTO** — po Q1 je redundantní, ale ponechán záměrně (rozhodnutí uživatele); A4 se ho drží. |
| Q9 | `str(payload)` alokuje kopii | **ZAMÍTNUTO** — `str()` na `str` vrací tentýž objekt. |
| Q10 | `deque` místo `pop(0)` | **ZAMÍTNUTO** — MicroPython `deque` vyžaduje `maxlen`, indexaci `[0]` nemá spolehlivě; 200 prvků = mikrosekundy v 1 s cyklu. |
| Q11 | Chybějící `ERR_WIFI`/`ERR_MQTT`/`ERR_AUTH` v `ERR_MQTT_MSG` | **ZAMÍTNUTO** — `last_error` se plní jen z `read_adc_channel()`/`convert_raw()`, které vracejí výhradně `ERR_CFG`/`ERR_ADC`/`ERR_LO`/`ERR_HI`/`ERR_NC`. |
| Q12 | `ntptime.timeout` je globální mutace | **ZAMÍTNUTO** — modulová proměnná je jediné API `ntptime`; `_sync_ntp()` je jediný volající. |
| Q13 | `gc.collect()` každý cyklus | **ZAMÍTNUTO** — záměr na 264 KB heapu, jednotky ms proti 1 s cyklu. |
| Q14 | `SharedResources` je God Object, rozdělit `config.py` | **ZAMÍTNUTO** — u `SharedResources` si to audit sám rozmyslel; jeden konfigurační soubor je záměr (`CLAUDE.md`). |

---

## Audit #6 — kritický review (verze 1.3.0)

**Datum:** 2026-04-18
**Soubory:** `main.py`
**Zaměření:** Potenciální chyby (WDT, síť, time zone) a sporná designová rozhodnutí

### Souhrn závažnosti

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| A1 | ~~VYSOKÁ~~ | WDT starvation v `flush_buffer()` — feed jen každých 10 zpráv × 2s timeout | **OPRAVENO** (v1.3.1) |
| A2 | ~~VYSOKÁ~~ | WDT starvation v `_publish_all()` — 0 feedů mezi N publish × 2s timeout | **OPRAVENO** (v1.3.1) |
| A3 | ~~VYSOKÁ~~ | `MQTTClient.connect()` bez timeoutu — responsivní TCP + mrtvý broker blokuje do WDT resetu | **OPRAVENO** (v1.5.2) |
| A4 | STŘEDNÍ | `getaddrinfo()` bez uživatelského timeoutu — DNS blok až ~5s | Otevřeno |
| A5 | ~~STŘEDNÍ~~ | Timezone mismatch Pico (local bez tz) vs. Postgres daemon (UTC s tz) | **OPRAVENO** (v1.4.0) |
| A6 | ~~NÍZKÁ~~ | `_offset_day` inicializován z pre-NTP RTC v `__init__` | **OPRAVENO** (v1.4.0) |
| D1 | DESIGN | `_mqtt_auth_err` zaseknutý natrvalo — reset jen rebootem | Otevřeno |
| D2 | DESIGN | Per-sensor `{topic}/status` s `retain=False` — nečitelné pro pozdější subscribery | Otevřeno |
| D3 | DESIGN | `_uptime_s` drift — přičítá nominální `INTERVAL_S`, ne skutečný elapsed | Otevřeno |
| D4 | DESIGN | `_pub_buffer` přístup zvenčí — encapsulation leak | Otevřeno |
| D5 | DESIGN | `create_sensor` tichá chyba — jen `print`, bez displeje/diag | Otevřeno |
| D6 | DESIGN | `BaseException` catch v `connect_mqtt()` příliš široký | Otevřeno |
| D7 | DESIGN | `show_value` ignoruje `precision` — nesoulad s MQTT formátem | Otevřeno |

---

### A1 — WDT starvation v `flush_buffer()` (VYSOKÁ) — OPRAVENO ve v1.3.1

**Soubor:** `main.py:366–377`

```python
while self._pub_buffer:
    if self.wdt and sent % 10 == 0:
        self.wdt.feed()
    ...
    self.mqtt.publish(topic, payload, retain=retain)  # socket timeout 2s
```

WDT se krmí před každou 10. iterací. Mezi krmeními: 10 × publish × až 2s (socket timeout) = až 20s bez feedu. Při `WDT_TIMEOUT_MS ≈ 8000` → reset. V praxi publish obvykle <100ms, ale pomalá/nestabilní síť to shodí. Tento nález je pokračování N3/R1 z auditů #4 a #5 — stále otevřený.

**Doporučení:** feedovat v každé iteraci, ne každé 10.

```python
while self._pub_buffer:
    if self.wdt:
        self.wdt.feed()
    ...
```

---

### A2 — WDT starvation v `_publish_all()` (VYSOKÁ) — OPRAVENO ve v1.3.1

**Soubor:** `main.py:621–627`, `main.py:351–360`

`_publish_all()` prochází senzory, každý volá `SensorChannel.publish()` → 2× `shared.publish()` (status + value). `shared.publish()` WDT nekrmí. `connect_mqtt()` krmí jen při reconnect cestě. Systémově: 4 senzory × 2 publish × 2s timeout = 16s potenciálně bez WDT feedu.

**Doporučení:** přidat `self.wdt.feed()` na začátek `shared.publish()`:

```python
def publish(self, topic, payload, retain=True):
    if self.wdt:
        self.wdt.feed()
    if not self.connect_mqtt():
        return False
    ...
```

---

### A3 — `MQTTClient.connect()` bez timeoutu (VYSOKÁ) — OPRAVENO ve v1.5.2

**Soubor:** `main.py:307–308`

```python
client.connect()                   # blokuje dokud CONNACK, bez timeoutu
client.sock.settimeout(2)          # až poté
```

TCP pre-test (řádky 275–290) chytí nedostupného brokera. Ale **responsivní TCP + mrtvý MQTT broker** (broker přijme TCP syn-ack, nepošle CONNACK) → `client.connect()` blokuje do WDT resetu.

**Původní doporučení (monkey-patch `client.sock = sock` s vlastním socketem) bylo chybné** a vycházelo z nesprávného tvrzení, že umqtt.simple nemá veřejné API pro socket timeout. `connect()` si `self.sock` vytváří sám a přepsal by ho. Skutečná signatura:

```python
def connect(self, clean_session=True, timeout=None):
    self.sock = socket.socket()
    self.sock.settimeout(timeout)   # PŘED sock.connect() i před čtením CONNACK
```

**Oprava (v1.5.2):** `client.connect(timeout=4)`. Pokrývá oba scénáře — SYN bez odpovědi i responzivní TCP bez CONNACK. Řádek `client.sock.settimeout(2)` zůstává, nastavuje provozní timeout pro publish/ping. TCP pre-test se nemazal.

---

### A4 — `getaddrinfo()` bez uživatelského timeoutu (STŘEDNÍ)

**Soubor:** `main.py:276`

`usocket.getaddrinfo(broker, port)[0][-1]` je blokující s default OS timeoutem (~5s). Při flaky DNS se to sčítá s 4s TCP pre-testem + potenciálním umqtt `connect()` (viz A3). Při WDT ~8s borderline.

**Doporučení:** cachovat první úspěšný lookup:

```python
if self._cached_broker_addr is None:
    self._cached_broker_addr = usocket.getaddrinfo(...)[0][-1]
addr = self._cached_broker_addr
```

Alternativně používat přímo IP v configu.

---

### A5 — Timezone mismatch Pico ↔ Postgres (STŘEDNÍ) — OPRAVENO ve v1.4.0

**Soubor:** `main.py:449–451`, `postgres-push/mqtt_to_postgres.py:132`

Pico vytváří timestamp v **lokálním čase (CET/CEST) bez tz markeru**:

```python
t = time.gmtime(time.time() + self.shared.utc_offset)
ts = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}".format(...)
```

Postgres daemon fallback (pro zprávy bez ts) posílá UTC s `+00:00`:

```python
ts = " ".join(parts[1:]) or datetime.now(timezone.utc).isoformat()
```

Pokud je DB sloupec `TIMESTAMPTZ` a session TZ = UTC, Postgres interpretuje Pico string jako UTC → **posun o 1–2h**. Pokud je sloupec `TIMESTAMP` (bez tz), se Pico ts OK, ale fallback s `+00:00` se konvertuje do session TZ → také posun.

**Doporučení:** Pico má posílat UTC ISO bez offsetu (daemon pak přidá jednotně), nebo obě strany s explicitním `+HH:MM`:

```python
# Pico — varianta UTC:
t = time.gmtime()
ts = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(...)
```

Nutno ověřit typ sloupce `sensor_value.create_tms` v DB a session TZ daemonu.

---

### A6 — `_offset_day` inicializován z pre-NTP RTC (NÍZKÁ) — OPRAVENO ve v1.4.0

**Soubor:** `main.py:84–86`

```python
t = time.gmtime()                  # RTC ještě nesynchronizován
self.utc_offset = cet_offset(t)
self._offset_day = t[2]
```

V `__init__` je to voláno před `connect_wifi()` → NTP. RTC po bootu je typicky 1.1.2021 (RP2350 default). `cet_offset` pro měsíc 1 dá 3600s (CET), což je pro lednové datum správně, ale `_offset_day` je pak bogus. `connect_wifi` → `_sync_ntp` hned přepíše.

Pokud **NTP selže** (nedostupný server), zůstane bogus `_offset_day` až do úspěšného NTP. Daily resync v main-loopu čeká na `t[3] >= 1 and t[2] != _offset_day` — podmínka s bogus `_offset_day` se splní první iterací po NTP syncu a přepočítá. V nejhorším scénáři (NTP nikdy neuspěje) běží celou dobu s utc_offset=3600, timestamps jsou 1.1.2021+. Degradace, ne bezpečnostní problém.

**Doporučení:** počkat s inicializací `_offset_day` až po prvním úspěšném `_sync_ntp()` (nebo ji vynechat, resync logika se stejně spoléhá na NTP).

---

### D1 — `_mqtt_auth_err` zaseknutý natrvalo (DESIGN)

**Soubor:** `main.py:120, 262–263, 328`

Jednou nastavené `_mqtt_auth_err = True` nikdy nespadne zpět na False. Po opravě hesla v configu a nahrání přes `mpremote cp` musí uživatel ručně resetnout zařízení. Intuitivní chování: zkusit znovu po WiFi reconnectu (změna configu typicky vyžaduje reboot, ale při ponechání zařízení online mezi opravami se hodí).

**Doporučení:** v `check_wifi()` větvi reconnect resetovat flag:

```python
if self.wlan.isconnected() and self._wifi_connecting:
    self._mqtt_auth_err = False  # retry po reconnectu
    ...
```

---

### D2 — Per-sensor status s `retain=False` (DESIGN)

**Soubor:** `main.py:444`

```python
if not self.shared.publish(self.status_topic, status, retain=False):
```

Subscribery, kteří připojí až po publikaci, neuvidí `{topic}/status` dokud zařízení nepošle další hodnotu (`INTERVAL_S` sec). LWT "offline" je retained jen pro globální `MQTT_STATUS_TOPIC`, ne per-sensor. Dashboardy čtoucí "last known" nezobrazí nic.

**Doporučení:** zvážit `retain=True` pro status. Nevýhoda: při restartu brokeru zůstane stará hodnota do prvního publishe.

---

### D3 — `_uptime_s` drift (DESIGN)

**Soubor:** `main.py:684`

```python
self._uptime_s += config.INTERVAL_S
```

Přičítá nominální interval, ne skutečný elapsed. Když loop overruns (backoff, reconnect, flush_buffer), uptime zaostává. Po pár dnech desítky minut off.

**Doporučení:** použít `time.ticks_ms` s overflow-safe logikou (vzor z commit `3ce0819`):

```python
now = time.ticks_ms()
delta = time.ticks_diff(now, self._last_tick)
self._uptime_ms += delta
self._last_tick = now
# _uptime_s = _uptime_ms // 1000
```

---

### D4 — `_pub_buffer` encapsulation leak (DESIGN)

**Soubor:** `main.py:454–457`, `main.py:635`

`SensorChannel.publish()` a `SensorManager._publish_diag()` sahají na `self.shared._pub_buffer` přímo (underscore prefix + externí přístup). Kód funguje, ale je křehký — změna reprezentace `_pub_buffer` (např. na deque) by musela upravit více míst.

**Doporučení:** metoda na `SharedResources`:

```python
def enqueue(self, topic, payload, retain):
    if len(self._pub_buffer) >= config.PUBLISH_BUFFER_MAX:
        self._pub_buffer.pop(0)
    self._pub_buffer.append((topic, payload, retain))
```

---

### D5 — `create_sensor` tichá chyba (DESIGN)

**Soubor:** `main.py:556–567`

```python
except (KeyError, ValueError) as e:
    print("Chybná konfigurace senzoru:", e)
    return None
```

Senzor chybí ve výsledném seznamu, displej nezobrazí chybu, MQTT nevidí nic. Jen `print` do sériové konzole, kam se nikdo nekouká. Provozní follow-up po aktualizaci configu může trvat týdny, než si někdo všimne chybějícího kanálu.

**Doporučení:** minimálně `show_error(ERR_CFG)` + publikovat do diag topicu. Agresivnější: odmítnout boot, zobrazit `E--4` natrvalo (visible error > silent degraded).

---

### D6 — Široký `BaseException` v `connect_mqtt()` (DESIGN)

**Soubor:** `main.py:317–324`

```python
except BaseException as e:
    if isinstance(e, (MemoryError, KeyboardInterrupt, SystemExit)):
        raise
```

Proč ne `except Exception`? `BaseException` + ruční re-raise tří konkrétních typů je defenzivní vzor, který naznačuje nejistotu o tom, co umqtt může hodit.

**Doporučení:** zúžit na `(OSError, MQTTException)`; ostatní nechat propadnout do top-level FATAL handleru (to je od toho).

---

### D7 — `show_value` ignoruje `precision` (DESIGN)

**Soubor:** `main.py:381–398`, vs. `main.py:452`

Display formátuje podle magnitudy (0–9.999, 10–99.99, 100–999.9), ignoruje `self.precision` konfiguraci. MQTT publikuje `"{:.{precision}f} {ts}"`. Pro uživatele sledujícího displej i MQTT inkonzistentní.

**Doporučení:** buď dokumentovat záměr ("display má pevných 4 digits, MQTT má konfigurovatelnou precision"), nebo propojit — použít `precision` i pro display (s fallbackem na magnitude-based když se do 4 digits nevejde).

---

### Ověřená správnost (v1.3.0)

| Oblast | Výsledek |
|--------|----------|
| `cet_offset` — přechody CET/CEST včetně boundary při 01:00 UTC | Správně ✓ |
| `_last_sunday` — výpočet poslední neděle daného měsíce | Správně ✓ |
| MQTT auth podpora (`user`, `password`) + CONNACK rc 4/5 handling | Správně ✓ |
| `_mqtt_ever_connected` — reconn_mqtt neinkrementuje při prvním connect | Správně ✓ |
| `ticks_diff` polarita v `check_wifi`, `connect_mqtt`, `mqtt_ping` | Správně ✓ |
| `_close_mqtt` — fallback na `sock.close()` když `disconnect()` selže | Správně ✓ |
| `_btn_pressed` — debounce 200ms přes `ticks_diff` | Správně ✓ |
| `flush_buffer` — drop-oldest při plném bufferu (keep recent) | Správně ✓ |
| `PiraniSensor` — validace `u_divider`, `u_min/u_max`, `p_min/p_max` v `__init__` | Správně ✓ |
| `CurrentLoopSensor` — dead-zone `v_min * 0.8` + overshoot `v_max * 1.1` | Správně ✓ |
| `ADS1115` průměrování přes `sum(vals) // len(vals)` | Správně ✓ |
| Per-sensor exponenciální backoff `1000ms → 60000ms` (ADC, MQTT) | Správně ✓ |

### Stav předchozích otevřených nálezů

| ID | Popis | Stav |
|----|-------|------|
| S1 | MQTT bez autentizace | **OPRAVENO** (v1.3.0 — `MQTT_USER`/`MQTT_PASSWORD` v configu) |
| S2 | WiFi credentials plaintext | Otevřeno (omezení platformy) |
| S3 | Diagnostika bez access control | Otevřeno (mitigováno S1 fixem) |
| S4 | `mqtt_to_postgres` diag JSON tiše zahozen | Otevřeno |
| N3/R1 | `flush_buffer` WDT starvation | Přejmenováno na **A1**, stále otevřeno |
| N4 | `show_value` display overflow pro hraniční hodnoty | **OPRAVENO** (v1.5.1) |
| N6 | `ntptime.settime()` 5s blok bez WDT feed | Otevřeno (volá se z `__init__` před WDT startem + z `check_wifi` kde WDT běží — borderline při `WDT_TIMEOUT_MS=8000`) |

---
---

## Nálezy přenesené ze starších kol (stále otevřené)

Kola #1–#5 (2026-03-21 → 2026-04-11) byla z tohoto souboru vypuštěna: jejich opravené
nálezy jsou zaznamenané v `CHANGELOG.md`. Co z nich zůstalo otevřené, je zde.

| ID (kolo) | Závažnost | Popis | Poznámka |
|-----------|-----------|-------|----------|
| S2 (#5) | STŘEDNÍ | WiFi credentials v `config.py` jako plaintext | Omezení platformy; `config.py` je v `.gitignore` |
| S3 (#5) | NÍZKÁ | Diagnostika na `DIAG_TOPIC` zveřejňuje interní stav bez access control | Mitigováno fixem S1 (MQTT auth ve v1.3.0) |
| S4 (#5) | NÍZKÁ | `mqtt_to_postgres` tiše zahazuje diag JSON jako unparseable zprávu | Totožné s B6 v `postgres-push/AUDIT.md` |
| N6 (#4) | NÍZKÁ | `ntptime.settime()` blokuje až 5 s bez WDT feedu | `_sync_ntp()` — volá se z `__init__` (před startem WDT) i z `check_wifi()` (WDT běží); borderline při `WDT_TIMEOUT_MS=8000`. **Zmírněno v1.5.3**: feed před každým pokusem a `ntptime.timeout` 2 s. Zbývá DNS dotaz na `pool.ntp.org`, který žádný timeout neomezuje: pomalé DNS + 2 s může přesáhnout 8 s, a před prvním NTP se pokus opakuje každých ~10 min. Nepovinná volba `NTP_HOST` (IP adresa) riziko odstraní; výchozí zůstává `pool.ntp.org` (rozhodnutí uživatele 2026-10-20). |
| B3 (#3) | INFO | `gc.collect()` se přeskočí při přetažené iteraci (`remaining <= 0`) | Konec `run()`; MicroPython spouští GC i při selhání alokace, samoléčí se |
| D1 (#1) | INFO | Status topic se při výpadku MQTT nebufferuje | `SensorChannel.publish()` — záměr, ale není okomentováno |
| D2 (#1) | INFO | `_pub_buffer.pop(0)` je O(n) | `flush_buffer()` — přijatelné pro `PUBLISH_BUFFER_MAX=200` |

## Historie — uzavřené nálezy

Detaily jsou v `CHANGELOG.md` u příslušné verze.

| ID (kolo) | Popis | Stav |
|-----------|-------|------|
| F0–F3 (#1) | Validace Pirani configu, paho 1.x/2.x kompatibilita, nedefinovaný `client`, nesoulad README | OPRAVENO v 1.1.4 |
| A2–A6 (#2) | `utc_offset` po NTP, `ValueError` z `PiraniSensor`, `getaddrinfo()` timeout, `sock.settimeout()`, `retain` pro status topic | OPRAVENO v 1.2.1–1.2.3 |
| A1 (#2) | WDT nespustí při selhání `__init__` | UZAVŘENO — záměrné chování |
| B1, B2 (#3) | NTP retry každou sekundu po celý den; socket únik při selhání `disconnect()` | OPRAVENO v 1.2.4 / 1.2.5 |
| N3/R1 (#4/#1) | `flush_buffer` WDT starvation | OPRAVENO v 1.3.1 (jako A1 v kole #6) |
| N4 (#4) | `show_value` display overflow pro hraniční hodnoty | OPRAVENO v 1.5.1 |
| N5/D4 (#2 pg) | `db_worker` backoff neresetován po insertu | FALSE POSITIVE |
| S1 (#5) | MQTT bez autentizace | OPRAVENO v 1.3.0 (`MQTT_USER`/`MQTT_PASSWORD`) |
| S5 (#5) | `ticks_ms` uptime overflow po ~12.4 dnech | OPRAVENO |
