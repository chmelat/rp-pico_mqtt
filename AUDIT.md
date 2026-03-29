# Audit zpráva — rp-pico_mqtt

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
| A5  | NÍZKÁ       | `client.sock.settimeout()` — přístup k privátnímu API umqtt | Otevřeno |
| A6  | NÍZKÁ       | `retain=False` pro `status_topic` — rozporuje CLAUDE.md | Otevřeno |

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

#### A5 — `client.sock.settimeout()` — privátní API `umqtt.simple`

**Soubor:** `main.py:287`

`umqtt.simple.MQTTClient` po `connect()` exponuje atribut `.sock`. Ten není součástí veřejného API. Při aktualizaci knihovny selže tiše — `AttributeError` zachytí obecný `BaseException` handler a interpretuje to jako selhání připojení.

**Doporučení:** Okomentovat jako záměrný hack se zdůvodněním.

---

#### A6 — `retain=False` pro `status_topic` — rozporuje dokumentaci

**Soubor:** `main.py:415`

`CLAUDE.md` dokumentuje: *"both `retain=True`"*. Kód používá `retain=False` pro status. Subscriber připojující se po posledním publish neuvidí aktuální stav senzoru.

Jde buď o záměrnou odchylku (pak opravit dokumentaci), nebo o bug (pak opravit kód).

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
