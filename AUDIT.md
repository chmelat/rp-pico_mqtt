# Audit zpráva — rp-pico_mqtt

---

## Audit #3 — dlouhodobý provoz (verze 1.2.3)

**Datum:** 2026-03-29
**Zaměření:** Potenciální problémy při nepřetržitém provozu (memory, reconnect logika, časovače)

### Souhrn závažnosti

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| B1 | STŘEDNÍ   | NTP selhání → `_offset_day` neaktualizován → retry každou sekundu po celý den | **OPRAVENO v 1.2.4** |
| B2 | NÍZKÁ     | Socket únik při selhání `disconnect()` — přechodný, GC opraví | Otevřeno |
| B3 | INFO      | `gc.collect()` přeskočen při přetažené iteraci smyčky | Otevřeno |

---

### B1 — NTP selhání způsobí opakování každou sekundu po celý den — OPRAVENO v 1.2.4

**Soubor:** `main.py:641–646`

Pokud `_sync_ntp()` selhala, výjimka byla zachycena uvnitř a `_offset_day` se neaktualizoval. Podmínka v `run()` platila každou sekundu po celý zbytek dne — každý pokus blokoval smyčku až 5 sekund.

`_sync_ntp()` nyní vrací `True`/`False`. Volající v `run()` aplikuje fallback (lokální DST výpočet + aktualizace `_offset_day`) pokud WiFi není dostupná nebo NTP selže. Retry nastane až příští den.

---

### B2 — Socket únik při selhání `disconnect()`

**Soubor:** `main.py:237–244`

`umqtt.simple.MQTTClient.disconnect()` nejprve zapíše DISCONNECT paket, pak zavře socket. Pokud `sock.write()` vyhodí `OSError` (přerušené spojení), `sock.close()` se nikdy nezavolá. `_close_mqtt()` výjimku pohltí — socket zůstane otevřený až do GC.

MicroPython lwIP pool má typicky 4–5 socketů. Při prudce nestabilní síti a zpožděném GC může pool přechodně přetéct, čímž `connect_mqtt()` selže s `ENOMEM`. Stav je přechodný — GC ho opraví při příštím volání `gc.collect()`.

**Doporučení:**

```python
def _close_mqtt(self):
    if self.mqtt is not None:
        try:
            self.mqtt.disconnect()
        except Exception:
            try:
                self.mqtt.sock.close()  # záloha při selhání disconnect()
            except Exception:
                pass
        self.mqtt = None
```

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
