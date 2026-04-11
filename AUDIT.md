# Audit zpráva — rp-pico_mqtt

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
