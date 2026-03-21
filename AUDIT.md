# Audit zpráva — rp-pico_mqtt

**Datum:** 2026-03-21
**Verze main.py:** 1.1.3
**Verze mqtt_to_postgres.py:** 1.0.0
**Soubory:** `main.py`, `postgres-push/mqtt_to_postgres.py`

---

## Souhrn závažnosti

| ID | Závažnost | Soubor | Popis |
|----|-----------|--------|-------|
| B1 | STŘEDNÍ | `main.py` | `PiraniSensor` — uncaught `ValueError` z `math.sqrt` může vést k WDT resetu |
| R1 | NÍZKÁ | `main.py` | `flush_buffer` — WDT feed jen každých 10 zpráv |
| D1 | INFO | `main.py` | Status topic se neukládá do bufferu při výpadku MQTT |
| D2 | INFO | `main.py` | `_pub_buffer.pop(0)` je O(n) |
| D3 | INFO | `mqtt_to_postgres.py` | Chybí autentizace MQTT |
| D4 | INFO | `mqtt_to_postgres.py` | Backoff se neresetuje po selhání insertu, jen po reconnectu |

---

## Opraveno od předchozího auditu

### F1 — `mqtt_to_postgres.py`: kompatibilita s `paho-mqtt >= 2.x`

**Stav:** OPRAVENO

Callback `on_connect` byl upraven na kompatibilní podpis s `*args`, takže skript funguje s `paho-mqtt` 1.x i 2.x.

### F2 — `main.py`: potenciální použití nedefinované proměnné `client`

**Stav:** OPRAVENO

V `connect_mqtt()` je nyní `client` inicializováno na `None` před `try` blokem a `disconnect()` se volá jen pokud byl klient vytvořen.

### F3 — dokumentace

**Stav:** OPRAVENO

Byly srovnány nalezené nesoulady v `README.md` a `postgres-push/README_mqtt_to_postgres.md` s aktuálním stavem implementace.

---

## Otevřené nálezy

### B1 — `PiraniSensor`: nezachycená `ValueError` z `math.sqrt`

**Soubor:** `main.py`
**Závažnost:** STŘEDNÍ

```python
u_actual = voltage * self.u_divider
pressure = math.exp(self.a + self.b * u_actual + self.c * math.sqrt(u_actual))
```

**Problém:**
Pokud je `u_divider` nakonfigurováno jako záporné číslo, `u_actual` je záporné i při nezáporném `voltage`. `math.sqrt` na záporném čísle hodí `ValueError`. V bloku je zachycen pouze `OverflowError`. Výjimka probublá přes `read()` až do hlavní smyčky a skončí ve fatal větvi s následným WDT resetem.

**Oprava:**
```python
try:
    u_actual = voltage * self.u_divider
    pressure = math.exp(self.a + self.b * u_actual + self.c * math.sqrt(u_actual))
except (OverflowError, ValueError):
    return None, ERR_HI
```
Nebo validovat `u_divider > 0` už v `PiraniSensor.__init__` a vracet `ERR_CFG`.

---

## Problémy spolehlivosti

### R1 — `flush_buffer`: WDT se krmí jen každých 10 zpráv

**Soubor:** `main.py`
**Závažnost:** NÍZKÁ

```python
if self.wdt and sent % 10 == 0:
    self.wdt.feed()
```

**Problém:**
Při větším zásobníku a vyšší latenci MQTT může `10 zpráv × latence` překročit `WDT_TIMEOUT_MS`.

**Doporučení:**
Krmit WDT častěji, nebo podle uplynulého času místo pevného počtu zpráv.

---

## Designové poznámky

### D1 — Status topic se neukládá do bufferu při výpadku MQTT

**Soubor:** `main.py`

Status zpráva se při selhání publish nezapisuje do `_pub_buffer`, zatímco hodnota ano. Stav senzoru se tedy může při krátkém výpadku ztratit a napraví se až dalším měřicím cyklem. Pokud je to záměr, stálo by za to to explicitně okomentovat v kódu.

### D2 — `_pub_buffer.pop(0)` je O(n)

**Soubor:** `main.py`

Pro aktuální limit `PUBLISH_BUFFER_MAX = 200` je to přijatelné, ale při růstu kapacity by bylo vhodnější použít datovou strukturu s O(1) odebíráním zleva.

### D3 — Chybí autentizace MQTT v daemonu

**Soubor:** `postgres-push/mqtt_to_postgres.py`

Současný bridge používá `mqtt.Client()` bez `username/password`. V izolované síti to může být záměr, ale v méně důvěryhodném prostředí by bylo vhodné doplnit volitelnou autentizaci do konfigurace.

### D4 — Backoff v `db_worker` se neresetuje po úspěšném insertu

**Soubor:** `postgres-push/mqtt_to_postgres.py`

`backoff` se resetuje po úspěšném připojení k DB. Funkčně to stačí, ale po selhání insertu by bylo konzistentnější resetovat backoff i po následném úspěšném zápisu batchu.

---

## Ověřená správnost

Níže uvedené oblasti byly znovu prověřeny a jsou bez nálezu.

| Oblast | Výsledek |
|--------|----------|
| `_last_sunday` — výpočet poslední neděle | Správně ✓ |
| `cet_offset` — přechody CET/CEST | Správně ✓ |
| `show_value` — formátování na 4-digit TM1637 | Správně ✓ |
| ADC průměrování `sum(vals) // len(vals)` | Správně ✓ |
| `ticks_diff` polarita v reconnect logice | Správně ✓ |
| `mqtt_to_postgres.py` — `on_connect` kompatibilita s `paho-mqtt` 1.x i 2.x | Správně ✓ |
| `connect_mqtt()` — error path kolem `client` | Správně ✓ |
| README a bridge README vs. aktuální implementace | Synchronizováno ✓ |
