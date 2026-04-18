# Audit zpráva — rp-pico_mqtt

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
| A3 | VYSOKÁ | `MQTTClient.connect()` bez timeoutu — responsivní TCP + mrtvý broker blokuje do WDT resetu | Otevřeno |
| A4 | STŘEDNÍ | `getaddrinfo()` bez uživatelského timeoutu — DNS blok až ~5s | Otevřeno |
| A5 | STŘEDNÍ | Timezone mismatch Pico (local bez tz) vs. Postgres daemon (UTC s tz) | Otevřeno |
| A6 | NÍZKÁ | `_offset_day` inicializován z pre-NTP RTC v `__init__` | Otevřeno |
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

### A3 — `MQTTClient.connect()` bez timeoutu (VYSOKÁ)

**Soubor:** `main.py:307–308`

```python
client.connect()                   # blokuje dokud CONNACK, bez timeoutu
client.sock.settimeout(2)          # až poté
```

TCP pre-test (řádky 275–290) chytí nedostupného brokera. Ale **responsivní TCP + mrtvý MQTT broker** (broker přijme TCP syn-ack, nepošle CONNACK) → `client.connect()` blokuje do WDT resetu. umqtt.simple nemá veřejné API pro nastavení socket timeoutu před `connect()`.

**Doporučení:** patch umqtt nebo vlastní wrapper, který vytvoří socket s timeoutem:

```python
import usocket
sock = usocket.socket()
sock.settimeout(4)
addr = usocket.getaddrinfo(broker, port)[0][-1]
sock.connect(addr)
# pak předat do MQTTClient přes monkey-patch: client.sock = sock
```

Alternativa: zmenšit `MQTT_KEEPALIVE` + zvětšit `WDT_TIMEOUT_MS` (nepreferované).

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

### A5 — Timezone mismatch Pico ↔ Postgres (STŘEDNÍ)

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

### A6 — `_offset_day` inicializován z pre-NTP RTC (NÍZKÁ)

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
| N4 | `show_value` display overflow pro hraniční hodnoty | Otevřeno (viz D7) |
| N6 | `ntptime.settime()` 5s blok bez WDT feed | Otevřeno (volá se z `__init__` před WDT startem + z `check_wifi` kde WDT běží — borderline při `WDT_TIMEOUT_MS=8000`) |

---

## Audit #5 — bezpečnostní audit (verze 1.2.6)

**Datum:** 2026-04-11
**Soubory:** `main.py`, `config.py.example`, `postgres-push/mqtt_to_postgres.py`
**Zaměření:** Bezpečnost, nové diagnostické funkce, stav předchozích nálezů

### Souhrn závažnosti

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| S1 | STŘEDNÍ | MQTT bez autentizace — kdokoliv v síti může publikovat falešná data | Otevřeno |
| S2 | STŘEDNÍ | WiFi credentials v `config.py` jako plaintext | Otevřeno (omezení platformy) |
| S3 | NÍZKÁ | Diagnostika zveřejňuje interní stav zařízení bez access control | Otevřeno |
| S4 | NÍZKÁ | `mqtt_to_postgres`: diag JSON tiše zahozen jako unparseable message | Otevřeno |
| S5 | ~~NÍZKÁ~~ | `ticks_ms` uptime overflow po ~12.4 dnech | **OPRAVENO** |
| N3 | NÍZKÁ-STŘEDNÍ | `flush_buffer` WDT starvation (z auditu #4) | Otevřeno |
| N4 | NÍZKÁ | `show_value` display overflow pro hraniční záporné hodnoty (z auditu #4) | Otevřeno |
| N6 | NÍZKÁ | `ntptime.settime()` blokuje až 5s bez WDT feed (z auditu #4) | Otevřeno |

---

### S1 — MQTT bez autentizace (STŘEDNÍ)

**Soubory:** `main.py:294–297`, `postgres-push/mqtt_to_postgres.py:184`

`MQTTClient` a `paho.mqtt.Client` se připojují bez username/password a bez TLS. Kdokoliv v lokální síti může:
- **Publikovat** falešné senzorové hodnoty na `sensor/+/+` — `mqtt_to_postgres` je uloží do DB bez ověření autenticity
- **Subscribovat** a číst senzorová data a diagnostiku
- **Publikovat** na `MQTT_STATUS_TOPIC` → falešný "offline" status

`umqtt.simple.MQTTClient` podporuje `user` a `password` parametry. Mosquitto podporuje ACL. TLS je na Pico 2W možný přes `ssl.wrap_socket()`.

**Riziko:** Závisí na síťové segmentaci. V izolované průmyslové síti nízké, ve sdílené síti střední.

**Doporučení:**
1. Přidat volitelné `MQTT_USER` / `MQTT_PASSWORD` do config
2. Na brokeru nastavit ACL (pico smí jen publish na `sensor/<id>/#`, mqtt_to_postgres smí subscribe)

---

### S2 — WiFi credentials jako plaintext (STŘEDNÍ)

**Soubor:** `config.py`

SSID a heslo jsou v `config.py` jako plaintext. `config.py` je v `.gitignore` (šablona je `config.py.example`), ale:
- Při kopírování `mpremote cp` je heslo viditelné v historii shellu
- Na Pico filesystem je čitelné přes `mpremote` bez autentizace (fyzický USB přístup)

**Omezení platformy:** MicroPython nemá keystore/secure storage API. Pico 2W nemá hardware secure element.

**Doporučení:** Přijatelné riziko pro embedded zařízení s fyzickým přístupem. Zajistit, že `config.py` je v `.gitignore`.

---

### S3 — Diagnostika zveřejňuje interní stav (NÍZKÁ)

**Soubor:** `main.py:616–633`

`_publish_diag()` publikuje na MQTT:
- `mem` — volná paměť (informace o zátěži zařízení)
- `reconn_wifi` / `reconn_mqtt` — počet reconnectů (indikátor nestability sítě)
- `ver` — verze firmware (útočník zjistí, proti čemu útočí)

Bez MQTT autentizace (S1) jsou tato data čitelná kýmkoliv v síti.

**Riziko:** Nízké. Zařízení nemá shell, web server ani jiný vstupní vektor. Informace nemají praktickou hodnotu pro útok na Pico samotné. Relevantní pouze pokud S1 není vyřešeno.

---

### S4 — `mqtt_to_postgres` tiše zahazuje diag JSON (NÍZKÁ)

**Soubor:** `postgres-push/mqtt_to_postgres.py:133–142`

Topic `sensor/L200h/diag` nekončí na `/status`, takže projde filtrem (řádek 135). Payload `{"uptime":...}` selže na `float(parts[0])` → `ValueError` → `return`. Zpráva se tiše zahodí.

Funkčně neškodí — diag data nepatří do `sensor_value` tabulky. Ale zvyšuje objem tichých dropů (B6 z auditu #1).

**Doporučení:** Přidat filtr pro `/diag` topic vedle `/status`:

```python
if t.endswith("/status") or t.endswith("/diag"):
    return
```

---

### S5 — `ticks_ms` uptime overflow po ~12.4 dnech — OPRAVENO

**Soubor:** `main.py`

`ticks_ms()` na RP2350 je 30-bit. `ticks_diff` vrací zápornou hodnotu po ~12.4 dnech → diagnostický uptime by byl chybný.

**Řešení:** Nahrazeno vlastním čítačem `_uptime_s` v `SensorManager`, inkrementovaným o `INTERVAL_S` každou iteraci hlavní smyčky. MicroPython `int` je arbitrary precision — žádný overflow.

---

### Ověřená správnost (v1.2.6)

| Oblast | Výsledek |
|--------|----------|
| `config.py.example` — nové klíče `DIAG_INTERVAL`, `DIAG_TOPIC` | Správně ✓ |
| `_publish_diag()` — žádný blocking, žádná alokace mimo dict + json.dumps | Správně ✓ |
| `_publish_diag()` — RSSI čtení v try/except (neimplementováno na všech FW) | Správně ✓ |
| `reconn_wifi` — inkrementuje jen při reconnectu (ne při prvním connect) | Správně ✓ |
| `reconn_mqtt` — `_mqtt_ever_connected` flag zabraňuje počítání prvního connect | Správně ✓ |
| `_diag_counter` — korektní reset, `DIAG_INTERVAL=0` vypne diagnostiku | Správně ✓ |
| `json.dumps(diag)` — MicroPython `json` modul dostupný na RP2350 | Správně ✓ |
| `config.py` v `.gitignore` (credentials nejsou v repo) | Správně ✓ |
| Žádný user input v MQTT topic/payload (no injection vector) | Správně ✓ |
| `publish()` konvertuje payload přes `str()` — žádné binární injection | Správně ✓ |
| `mqtt_to_postgres` — parameterized SQL queries (žádné SQL injection) | Správně ✓ |
| `mqtt_to_postgres` — `float()` parsing odmítne non-numeric payload | Správně ✓ |

### Stav předchozích otevřených nálezů

| ID | Popis | Stav |
|----|-------|------|
| N3 | `flush_buffer` WDT starvation | Otevřeno (NÍZKÁ-STŘEDNÍ) |
| N4 | `show_value` overflow pro záporné hodnoty | Otevřeno (NÍZKÁ) |
| N6 | `ntptime.settime()` blokuje 5s bez WDT feed | Otevřeno (NÍZKÁ) |
| B3 | `gc.collect()` přeskočen při přetažené iteraci | Otevřeno (INFO) |
| D1 | Status topic se nebufferuje | Otevřeno (záměr) |
| D2 | `_pub_buffer.pop(0)` je O(n) | Otevřeno (OK pro limit 200) |
| B1 | `mqtt_to_postgres`: deque thread safety závisí na GIL | Otevřeno (Low) |
| B6 | `mqtt_to_postgres`: tiché zahazování špatných zpráv | Otevřeno (Low) — rozšířeno o S4 |
| B7 | `mqtt_to_postgres`: tiché zahazování při plné queue | Otevřeno (Low) |
| D3 | `mqtt_to_postgres`: chybí MQTT autentizace | Otevřeno (INFO) — souvisí s S1 |

---

## Audit #4 — hloubkový audit (verze 1.2.5)

**Datum:** 2026-04-03
**Soubory:** `main.py`

### Souhrn závažnosti

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| N3 | NÍZKÁ-STŘEDNÍ | `flush_buffer` WDT starvation: 10 × 2s socket timeout > 8s WDT | Otevřeno |
| N4 | NÍZKÁ | `show_value` display overflow pro hraniční záporné hodnoty | Otevřeno |
| N6 | NÍZKÁ | `ntptime.settime()` blokuje až 5s bez WDT feed (margín 3s) | Otevřeno |

---

### N3 — `flush_buffer` WDT starvation při pomalém brokeru (NÍZKÁ-STŘEDNÍ)

**Soubor:** `main.py:342–358`

WDT se krmí každých 10 odeslaných zpráv (line 347–348). Socket timeout je 2s (line 295). Pokud broker přijímá TCP ale odpovídá pomalu, 10 publish × 2s = 20s bez WDT feed > `WDT_TIMEOUT_MS` (8s) → neplánovaný restart.

R1 z auditu #1 — stále otevřeno, nyní lépe kvantifikováno.

**Doporučení:** Krmit WDT před každým publish:

```python
while self._pub_buffer:
    if self.wdt:
        self.wdt.feed()
    topic, payload, retain = self._pub_buffer[0]
```

---

### N4 — `show_value` overflow pro hraniční záporné hodnoty (NÍZKÁ)

**Soubor:** `main.py:360–377`

Pokud `value = -9.995`, `"{:.2f}".format(-9.995)` → `"-10.00"` (5 znaků pro 4 pozice displeje). Zaokrouhlení formátovacím řetězcem posune hodnotu přes hranici `-10`, kde se přepne na `"{:.1f}"`, ale formátování proběhlo ještě v původní větvi.

V praxi senzory nevracejí záporné hodnoty (p_min ≥ 0, pirani p_min = 1e-4). Problém nastane jen při konfiguraci záporného rozsahu.

---

### N6 — `ntptime.settime()` blokuje až 5s bez WDT feed (NÍZKÁ)

**Soubor:** `main.py:148–159`

`ntptime.timeout = 5` a pak `ntptime.settime()` blokuje celý timeout bez krmení WDT. 5s < 8s WDT timeout, ale margín je jen 3s.

**Doporučení:** Snížit `ntptime.timeout` na 3, nebo přidat WDT feed před volání.

---

### Ověřená správnost (v1.2.5)

| Oblast | Výsledek |
|--------|----------|
| `_last_sunday` — weekday aritmetika pro měsíc 3 a 10 | Správně ✓ |
| `cet_offset` — přechod CET↔CEST na poslední neděli v 01:00 UTC | Správně ✓ |
| `time.mktime` na Pico — bez TZ, `mktime` = inverse of `gmtime` | Správně ✓ |
| `ticks_ms()` aritmetika — všude `ticks_diff`/`ticks_add` | Správně ✓ |
| `_sync_ntp` aktualizuje `utc_offset` i `_offset_day` | Správně ✓ |
| `_close_mqtt` socket cleanup fallback | Správně ✓ |
| `connect_mqtt` — `BaseException` catch s re-raise kritických | Správně ✓ |
| `_btn_irq` → `micropython.schedule` — korektní ISR deferral | Správně ✓ |
| Buffer bounded na `PUBLISH_BUFFER_MAX` | Správně ✓ |

### Stav předchozích otevřených nálezů

| ID | Popis | Stav |
|----|-------|------|
| B3 | `gc.collect()` přeskočen při přetažené iteraci | Otevřeno (INFO) |
| R1 | `flush_buffer` WDT starvation | Otevřeno → upřesněno jako N3 |
| D1 | Status topic se nebufferuje | Otevřeno (záměr) |
| D2 | `_pub_buffer.pop(0)` je O(n) | Otevřeno (OK pro limit 200) |

---

## Audit #3 — dlouhodobý provoz (verze 1.2.3)

**Datum:** 2026-03-29
**Zaměření:** Potenciální problémy při nepřetržitém provozu (memory, reconnect logika, časovače)

### Souhrn závažnosti

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| B1 | STŘEDNÍ   | NTP selhání → `_offset_day` neaktualizován → retry každou sekundu po celý den | **OPRAVENO v 1.2.4** |
| B2 | STŘEDNÍ   | Socket únik při selhání `disconnect()` — potvrzeno v produkci (E--2, restart pomohl) | **OPRAVENO v 1.2.5** |
| B3 | INFO      | `gc.collect()` přeskočen při přetažené iteraci smyčky | Otevřeno |

---

### B1 — NTP selhání způsobí opakování každou sekundu po celý den — OPRAVENO v 1.2.4

**Soubor:** `main.py:641–646`

Pokud `_sync_ntp()` selhala, výjimka byla zachycena uvnitř a `_offset_day` se neaktualizoval. Podmínka v `run()` platila každou sekundu po celý zbytek dne — každý pokus blokoval smyčku až 5 sekund.

`_sync_ntp()` nyní vrací `True`/`False`. Volající v `run()` aplikuje fallback (lokální DST výpočet + aktualizace `_offset_day`) pokud WiFi není dostupná nebo NTP selže. Retry nastane až příští den.

---

### B2 — Socket únik při selhání `disconnect()` — OPRAVENO v 1.2.5

**Soubor:** `main.py:237–244`

`umqtt.simple.disconnect()` nezavře socket pokud `sock.write()` selže. Při nestabilní síti se lwIP pool (4–5 socketů) postupně vyčerpal, `connect_mqtt()` selhalo s `ENOMEM` → trvalé `E--2` i přes funkční měření. Potvrzeno v produkci — restart pomohl.

`_close_mqtt()` nyní explicitně zavírá socket v except větvi jako záloha.

---

### B3 — `gc.collect()` přeskočen při přetažené iteraci (INFO)

**Soubor:** `main.py:673`

```python
if remaining > 0:
    gc.collect()
```

Pokud iterace trvá déle než `INTERVAL_S * 1000 ms` (např. kvůli B1 nebo MQTT reconnectu), GC se nespustí. MicroPython spouští GC automaticky při selhání alokace, takže nejde o crash riziko. Při souběhu B1 (5s iterace) a aktivního bufferu může fragmentace narůstat rychleji. Samoléčí se při první úspěšné krátké iteraci.

---

### Ověřená správnost pro dlouhodobý provoz

| Oblast | Výsledek |
|--------|----------|
| `ticks_ms()` rollover (~49 dní) — všechna srovnání přes `ticks_diff` / `ticks_add` | Správně ✓ |
| `ticks_ms() % 2500` v `_update_display()` — Python `%` vrací vždy ≥ 0 | Správně ✓ |
| `_pub_buffer` — growth bounded na `PUBLISH_BUFFER_MAX` | Správně ✓ |
| WiFi reconnect deadline + retry logika | Správně ✓ |
| MQTT backoff — reset po reconnectu | Správně ✓ |
| Denní DST přepočet — správně čeká na 01:00 UTC | Správně ✓ |
| WDT krmení při `flush_buffer()` (každých 10 zpráv) | Přijatelné ✓ |

---

## Audit #2 — verze 1.2.0 → opraveno v 1.2.1

**Datum:** 2026-03-29
**Soubory:** `main.py`, `config.py.example`

### Souhrn závažnosti

| ID  | Závažnost   | Popis | Stav |
|-----|-------------|-------|------|
| A1  | ~~KRITICKÁ~~ | WDT nespustí při selhání `__init__` — zařízení nerestartuje | Není chyba — viz níže |
| A2  | KRITICKÁ    | `utc_offset` neaktualizován po NTP sync — chybná razítka až 24 h | **OPRAVENO v 1.2.1** |
| A3  | STŘEDNÍ     | `ValueError` z `PiraniSensor.__init__()` nekachytaná v `create_sensor()` | **OPRAVENO v 1.2.1** |
| A4  | STŘEDNÍ     | `getaddrinfo()` bez timeoutu — blokuje WDT pro hostname | **OPRAVENO v 1.2.2** |
| A5  | NÍZKÁ       | `client.sock.settimeout()` — přístup k privátnímu API umqtt | **OPRAVENO v 1.2.3** |
| A6  | NÍZKÁ       | `retain=False` pro `status_topic` — rozporuje CLAUDE.md | **OPRAVENO v 1.2.3** |

---

### Kritické chyby

#### A1 — WDT nespustí při selhání `__init__` — UZAVŘENO jako záměrné chování

**Soubor:** `main.py:627`

Selhání v `__init__` je téměř výhradně způsobeno chybnou konfigurací (`config.py`), která je statická. WDT reset by situaci nevyřešil — způsobil by nekonečnou reset smyčku bez možnosti diagnostiky. Zařízení zůstane na `E--5`, což je žádoucí: operátor vidí chybu a může ji opravit.

Oprava A3 (`ValueError` zachycena v `create_sensor()`) zároveň eliminuje nejpravděpodobnější příčinu pádu `__init__`.

---

#### A2 — `utc_offset` neaktualizován po NTP synchronizaci

**Soubor:** `main.py:79–80`, `main.py:148–155`, `main.py:168–181`

Pořadí inicializace v `SharedResources.__init__()`:

```python
t = time.gmtime()                # čas před NTP — může být epoch (2021-01-01)
self.utc_offset = cet_offset(t)  # vypočítán z epochy → špatný výsledek
...
self.connect_wifi()              # → _sync_ntp() → čas skočí na aktuální
```

Po NTP synchronizaci se `utc_offset` nepřepočítá. Zůstane špatný až do příštího přepočtu v hlavní smyčce, který nastane nejdříve po 01:00 UTC jiného dne (podmínka `t[3] >= 1 and t[2] != _offset_day`). V nejhorším případě jsou MQTT payloady s chybným časovým razítkem téměř 24 hodin.

Stejný problém nastane při WiFi reconnectu: `check_wifi()` volá `_sync_ntp()`, ale `utc_offset` neaktualizuje.

**Doporučení:** Přidat na konec `_sync_ntp()`:

```python
t = time.gmtime()
self.utc_offset = cet_offset(t)
self._offset_day = t[2]
```

---

### Střední závažnost

#### A3 — `ValueError` z `PiraniSensor.__init__()` nekachytaná

**Soubor:** `main.py:527–538`

`PiraniSensor.__init__()` hází `ValueError` pro neplatné parametry (přidáno v auditu #1 jako F0). `create_sensor()` zachytává pouze `KeyError`:

```python
except KeyError as e:
    print("Chybí klíč v konfiguraci senzoru:", e)
    return None
```

`ValueError` tedy propaguje do `SensorManager.__init__()` a pak do top-level handleru. V kombinaci s A1 způsobí trvalé zaseknutí zařízení. Záměr validace z F0 (vrátit `None` místo pádu) nebyl dotažen.

**Doporučení:** Zachytávat `(KeyError, ValueError)` v `create_sensor()`.

---

#### A4 — `getaddrinfo()` bez timeoutu v TCP pre-testu — OPRAVENO v 1.2.2

**Soubor:** `main.py:259`

Pro hostname by DNS dotaz bez timeoutu zablokoval vlákno na neurčito bez krmení WDT. MicroPython nenabízí `getaddrinfo()` s timeoutem.

Opraveno dokumentačním komentářem v `config.py.example` — `MQTT_BROKER` musí být vždy IP adresa.

---

### Nízká závažnost

#### A5 — `client.sock.settimeout()` — privátní API `umqtt.simple` — OPRAVENO v 1.2.3

**Soubor:** `main.py:290`

Přidán komentář přímo na řádek vysvětlující záměr a důvod přístupu k internímu atributu.

---

#### A6 — `retain=False` pro `status_topic` — OPRAVENO v 1.2.3

**Soubor:** `CLAUDE.md:66`

Záměrné chování — status se nepublikuje jako retained, aby subscriber po reconnectu nedostal zastaralý stav. Dokumentace v `CLAUDE.md` opravena tak, aby odpovídala kódu.

---

## Audit #1 — verze 1.1.4

**Datum:** 2026-03-21
**Soubory:** `main.py`, `postgres-push/mqtt_to_postgres.py`

### Opraveno

| ID | Popis | Stav |
|----|-------|------|
| F0 | `PiraniSensor`: validace konfigurace přidána do `__init__()` | OPRAVENO |
| F1 | `mqtt_to_postgres.py`: kompatibilita s `paho-mqtt` 1.x i 2.x (`on_connect` podpis) | OPRAVENO |
| F2 | `connect_mqtt()`: potenciální použití nedefinované proměnné `client` | OPRAVENO |
| F3 | Nesoulad `README.md` s implementací | OPRAVENO |

### Otevřené nálezy (přeneseno do auditu #2 nebo stále aktuální)

| ID  | Závažnost | Popis | Stav |
|-----|-----------|-------|------|
| R1  | NÍZKÁ     | `flush_buffer`: WDT feed jen každých 10 zpráv — při vysoké latenci MQTT může překročit `WDT_TIMEOUT_MS` | Otevřeno |
| D1  | INFO      | Status topic se neukládá do bufferu při výpadku MQTT (záměr, ale není okomentováno) | Otevřeno |
| D2  | INFO      | `_pub_buffer.pop(0)` je O(n) — přijatelné pro limit 200, problematické při růstu | Otevřeno |
| D3  | INFO      | `mqtt_to_postgres.py`: chybí volitelná MQTT autentizace | Otevřeno |
| D4  | INFO      | `mqtt_to_postgres.py`: backoff se neresetuje po úspěšném insertu, jen po reconnectu | Otevřeno |

### Ověřená správnost (v1.1.4)

| Oblast | Výsledek |
|--------|----------|
| `_last_sunday` — výpočet poslední neděle | Správně ✓ |
| `cet_offset` — přechody CET/CEST | Správně ✓ |
| `show_value` — formátování na 4-digit TM1637 (vč. záporných hodnot) | Správně ✓ |
| ADC průměrování `sum(vals) // len(vals)` | Správně ✓ |
| `ticks_diff` polarita v reconnect logice | Správně ✓ |
| `mqtt_to_postgres.py` — `on_connect` kompatibilita s `paho-mqtt` 1.x i 2.x | Správně ✓ |
| `connect_mqtt()` — error path kolem `client` | Správně ✓ |
| README a bridge README vs. aktuální implementace | Synchronizováno ✓ |
