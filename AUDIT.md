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
| N6 (#4) | NÍZKÁ | `ntptime.settime()` blokuje až 5 s bez WDT feedu | `_sync_ntp()` — volá se z `__init__` (před startem WDT) i z `check_wifi()` (WDT běží); borderline při `WDT_TIMEOUT_MS=8000` |
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
