# Ponytail audit — over-engineering

**Datum:** 2026-08-16
**Rozsah:** celý strom (`main.py` 1.5.0, `postgres-push/mqtt_to_postgres.py` 1.2.1, dokumentace)
**Zaměření:** výhradně zbytečná složitost — co smazat, zjednodušit nebo nahradit stdlib/platformou.
Správnost, bezpečnost a výkon jsou mimo rozsah (ty řeší `AUDIT.md`).

**Poměr:** 979 řádků kódu vs. 2231 řádků trackované dokumentace.

## Tagy

- `delete:` mrtvý kód, nepoužitá flexibilita, spekulativní feature
- `stdlib:` ručně psané to, co má standardní knihovna
- `native:` závislost nebo kód dělající to, co umí platforma
- `yagni:` abstrakce s jedinou implementací, config nikdo nenastavuje, vrstva s jedním volajícím
- `shrink:` stejná logika, méně řádků

## Stav

Vše vyřízeno k 2026-08-16 (`main.py` 1.5.1), detaily v `CHANGELOG.md`:

- **Aplikováno:** #1, #2, #3 (dokumentace) a #6, #8–#11, #13–#15 (kód).
- **Zamítnuto:** #4 (README je nadmnožina CLAUDE.md), #7 (neověřená verze paho na hostu
  démona), #12 (pojistka proti WDT starvation).
- **Mimo zadání:** #5 — behaviorální změna v postgres démonu, ne over-engineering.

Tenhle soubor už nemá co sledovat; po přečtení může jít pryč.

## Nálezy (řazeno podle velikosti řezu)

| # | Tag | Co vyříznout | Náhrada | Místo | Stav |
|---|-----|--------------|---------|-------|------|
| 1 | `delete:` | AUDIT.md kola #1–#5 | nic — všechny nálezy jsou OPRAVENO/UZAVŘENO a už jsou v `CHANGELOG.md`; ponechat jen otevřené položky kola #6 | `AUDIT.md:310-721` | **APLIKOVÁNO 1.5.1** — kola #1–#5 nahrazena souhrnem, otevřené nálezy zachovány |
| 2 | `delete:` | celý `ANALYSIS_REPORT.md` | nic — jednorázový snapshot v1.3.0 z 2026-03-21, o měsíc později nahrazen auditem #6 | `ANALYSIS_REPORT.md` | **APLIKOVÁNO 1.5.1** — smazán |
| 3 | `delete:` | uzavřené nálezy (N1/N2/N5 — tabulka i těla) | ponechat jen otevřené, zbytek patří do CHANGELOG | `postgres-push/AUDIT.md` | **APLIKOVÁNO 1.5.1** |
| 4 | `delete:` | README §Architektura, §Rozšíření o nový typ senzoru, §Chybové kódy | doslovné duplikáty `CLAUDE.md`, který se navíc reálně načítá — nahradit odkazem | `README.md:98,503,534` | **ZAMÍTNUTO** — README sekce jsou nadmnožina CLAUDE.md, ne duplikát |
| 5 | `delete:` | row-by-row retry po selhání batche | `conn.rollback()` + jeden log řádek; atomicita batche je záměrný tradeoff (viz fix B2), tenhle fallback vrací zpět cestu částečného zápisu, kterou B2 odstranil | `postgres-push/mqtt_to_postgres.py:74-89` | mimo zadání |
| 6 | `shrink:` | čtyřvětvená kaskáda desetinných míst v `show_value` | `for p in (3,2,1): s = "{:.{}f}".format(value, p)` + `if len(s.replace('.','')) <= 4: break`, pak jediný `show()` — stejný výstup včetně hraničních hodnot | `main.py:356-373` | **APLIKOVÁNO 1.5.1** |
| 7 | `yagni:` | shim `_PAHO_V2` a žonglování `flags_or_rc`/`reason_code` v `on_disconnect` | podpora paho 1.x i 2.x pro démona na jednom známém stroji — připnout paho ≥ 2 v README a volat `mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)` přímo | `postgres-push/mqtt_to_postgres.py:20,150-155` | **ZAMÍTNUTO** — verze paho na hostu démona neověřená |
| 8 | `shrink:` | čtyři samostatné `raise ValueError` v `PiraniSensor.__init__` | jeden guard: `if u_divider <= 0 or not 0 <= u_min < u_max or p_max <= p_min: raise ValueError(...)` | `main.py:498-505` | **APLIKOVÁNO 1.5.1** |
| 9 | `yagni:` | `_read_all` a `_publish_all` | jednořádkové smyčky s jedním volajícím, oba v `run()` — inlinovat | `main.py:599-610` | **APLIKOVÁNO 1.5.1** |
| 10 | `shrink:` | `has_conn_error` / `has_valid_value` / `has_no_sensor_error` | tři názvy použité jednou, v jediném `if`: `if self._conn_error and s.last_value is not None and not s.last_error:` | `main.py:638-642` | **APLIKOVÁNO 1.5.1** |
| 11 | `yagni:` | `self.i_min` / `i_max` / `i_disconnect` | ukládají se, ale hned na dalším řádku se jen násobí `r_bocnik` — počítat `v_min`/`v_max`/`v_disconnect` rovnou z `cfg.get(...)` | `main.py:444-452` | **APLIKOVÁNO 1.5.1** |
| 12 | `delete:` | `if i % 10 == 9 and self.wdt: self.wdt.feed()` ve vzorkovací smyčce | `ADC_SAMPLES` je 5 a 10 vzorků při 64 SPS je 156 ms proti 8s watchdogu — nikdy se nespustí | `main.py:213-214` | **ZAMÍTNUTO** — pojistka proti WDT starvation při vysokém `ADC_SAMPLES` |
| 13 | `yagni:` | config klíč `status_topic` | nenastavuje ho `config.py` ani `config.py.example` — `self.status_topic = cfg["topic"] + "/status"` | `main.py:389` | **APLIKOVÁNO 1.5.1** |
| 14 | `delete:` | `self.name` | přiřazeno, nikdy nečteno — zrušit atribut i dva klíče `"name":` v configu | `main.py:390` | **APLIKOVÁNO 1.5.1** |
| 15 | `shrink:` | alias `uptime = self._uptime_s` | použit jednou, inlinovat do dictu | `main.py:614` | **APLIKOVÁNO 1.5.1** |

**Odhad: -743 řádků dokumentace, -55 řádků kódu, -0 závislostí.**
**Reálně vyřezáno (1.5.1): -32 řádků v `main.py`, -574 řádků dokumentace** (bez #4, #7, #12).

## Záměrně ponecháno

- **`implementation_notes.md`** — vydřená CYW43 lore (jednosměrný socket, PINGRESP), z kódu se nedá odvodit.
- **`SENSOR_TYPES` + `create_sensor()`** — factory se dvěma reálnými produkty řízenými configem; abstrakce se vyplatí.
