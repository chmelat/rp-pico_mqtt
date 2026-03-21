# Analyza kodu

Datum analyzy: 2026-03-21

## Rozsah

Byly provedeny cteni a kontrola souboru:

- `main.py`
- `postgres-push/mqtt_to_postgres.py`
- `README.md`
- `AUDIT.md`
- `config.py.example`
- `implementation_notes.md`

## Celkove hodnoceni

Repozitar je architektonicky cisty a na embedded MicroPython projekt pomerne dobre disciplinovany. Hlavni cast na Pico ma rozumne oddelenou hardwarovou a sitovou vrstvu v `SharedResources`, senzory jsou rozsiritelne pres dedicnost od `SensorChannel`, a je videt, ze autor resil i provozni detaily jako watchdog, Wi-Fi reconnect, MQTT keepalive a problem zombie socketu na CYW43.

Vedlejsi bridge `postgres-push/` je jednoduchy a ucelny: oddeluje prijem MQTT od zapisu do databaze pres frontu a worker vlakno. Kod je snadno citelny a nema zbytecnou slozitost.

## Silne stranky

- Jasne oddeleni zodpovednosti mezi `SensorManager`, `SharedResources` a jednotlivymi typy senzoru.
- Rozsiritelny navrh senzoru pres `SENSOR_TYPES` factory.
- Promyslena odolnost vuci sitovym vypadkum:
  - Wi-Fi reconnect logika
  - MQTT reconnect s exponential backoff
  - publish buffer pro neodeslane hodnoty
  - periodicky `ping()` + `check_msg()` kvuli detekci zombie spojeni
- Konzervativni pristup k watchdogu a blokujicim operacim.
- Dobra technicka dokumentace k provoznim problemum v `implementation_notes.md`.

## Nalezy

### 1. Stredni zavaznost: `PiraniSensor` muze shodit hlavni smycku

Soubor: `main.py`

Relevantni misto:

- `PiraniSensor.convert_raw()`
- radky kolem vypoctu `math.sqrt(u_actual)`

Popis:

V `PiraniSensor.convert_raw()` se zachytava pouze `OverflowError`, ale ne `ValueError`. Pokud bude `u_divider` v konfiguraci zaporne cislo, pak `u_actual` muze byt zaporne a `math.sqrt(u_actual)` vyhodi `ValueError`. Tato vyjimka se dostane az na vrchol aplikace, kde skonci ve fatal vetvi a zarizeni spadne do WDT resetu.

Dopad:

- padu hlavni smycky lze dosahnout cistou konfiguracni chybou
- chyba se neprojevi jako bezna senzorova chyba, ale jako fatal restart zarizeni

Doporuceni:

- zachytit i `ValueError`
- nebo validovat `u_divider > 0` uz pri konstrukci `PiraniSensor`

### 2. Stredni zavaznost: `mqtt_to_postgres.py` neni kompatibilni s `paho-mqtt >= 2.x`

Soubor: `postgres-push/mqtt_to_postgres.py`

Relevantni misto:

- callback `on_connect`

Popis:

Funkce `on_connect(client, userdata, flags, rc)` ma stary ctyrargumentovy podpis. V novych verzich `paho-mqtt` se callback vola s dalsim argumentem navic. Pokud je v systemu nainstalovana nova verze knihovny, muze callback skoncit `TypeError` a daemon prestane fungovat.

Dopad:

- bridge muze spadnout hned po pripojeni k brokeru
- zavislost na konkretni verzi knihovny neni v projektu formalne omezena

Doporuceni:

- pouzit kompatibilni podpis, napr. s `*args`
- pripadne explicitne pinovat zavislost na starsi verzi

### 3. Nizka zavaznost: mozne pouziti nedefinovane promenne `client`

Soubor: `main.py`

Relevantni misto:

- `SharedResources.connect_mqtt()`

Popis:

V `except` vetvi se vola `client.disconnect()`, ale promenna `client` je vytvorena az uvnitr `try` bloku. Pokud by konstrukce `MQTTClient(...)` sama selhala pred prirazenim, obsluha chyby sahne na nedefinovanou promennou. Vnejsi `except BaseException` to v praxi spolkne, ale je to slabsi misto implementace.

Dopad:

- mala pravdepodobnost
- zhorsuje to robustnost error path

Doporuceni:

- inicializovat `client = None` pred `try`
- `disconnect()` volat jen pokud je klient skutecne vytvoren

### 4. Nizka zavaznost: krmeni watchdogu pri flushi bufferu je ridke

Soubor: `main.py`

Relevantni misto:

- `SharedResources.flush_buffer()`

Popis:

Watchdog se pri vyprazdnovani `_pub_buffer` krmi jen jednou za 10 odeslanych zprav. Pri vyssi latenci brokeru nebo vetsim bufferu muze deset publish operaci trvat prilis dlouho vzhledem k `WDT_TIMEOUT_MS`.

Dopad:

- za urcitych sitovych podminek muze dojit k resetu behem obnovy spojeni

Doporuceni:

- krmit WDT casteji
- nebo rozhodovat podle uplynuleho casu, ne podle poctu zprav

### 5. Nizka zavaznost: dokumentace neni plne synchronni s implementaci

Soubory:

- `AUDIT.md`
- `README.md`
- `postgres-push/README_mqtt_to_postgres.md`

Popis:

Bylo nalezeno nekolik mensich nesouladu:

- `AUDIT.md` uvadi verzi `1.1.2`, zatimco `main.py` ma `VERSION = "1.1.3"`
- nektere defaulty v `README.md` se rozchazeji s `config.py.example`
- README pro PostgreSQL bridge tvrdi, ze timestamp generuje daemon, ale kod realne prebira timestamp z MQTT payloadu

Dopad:

- muze mast uzivatele pri nasazeni nebo ladeni
- zhorsuje duveru v dokumentaci jako zdroj pravdy

Doporuceni:

- srovnat README a audit s aktualnim stavem kodu
- urcit, zda timestamp ma byt zdrojem na Pico nebo na serveru, a sjednotit implementaci s dokumentaci

## Architektonicke zhodnoceni

### Pico cast

`main.py` je navrzen jako jednovlaknova hlavni smycka s centralnimi sdilenymi zdroji. To je pro MicroPython na Pico rozumna volba. Kod se vyhyba zbytecne komplikaci a drzi vsechny citlive provozni stavy v jednom miste.

Pozitivni je hlavne:

- opakovane pouziti jedne instance Wi-Fi, I2C, ADC a displaye
- osetreni chyb pri inicializaci ADC
- buffering MQTT hodnot pri vypadku spojeni
- logika pro zobrazeni spojovych chyb na displeji bez uplne ztraty hodnoty

### PostgreSQL bridge

`postgres-push/mqtt_to_postgres.py` je minimalisticky, ale funkcne dava smysl:

- MQTT callback pouze prijima a frontuje
- DB worker zpracovava frontu oddelene
- pri chybe insertu se batch vraci zpet do fronty
- reconnect k databazi pouziva exponential backoff

Je to dobry navrh pro jednoduchy daemon, i kdyz by do budoucna stalo za zvazeni:

- formalni sprava zavislosti
- volitelna MQTT autentizace
- presnejsi dokumentace kolem timestampu a shutdown semantics

## Zaver

Kod je v dobre kondici a neni to chaoticky prototyp. Nejvetsi technicka rizika jsou dve konkretni chyby:

1. pad `PiraniSensor` pri chybne konfiguraci
2. nekompatibilita PostgreSQL bridge s novou verzi `paho-mqtt`

Obe chyby jsou pomerne male na opravu, ale maji realny provozni dopad. Zbytek nalezu je spis o robustness detailu a synchronizaci dokumentace s implementaci.

## Doporucene dalsi kroky

1. Opravit `PiraniSensor.convert_raw()` tak, aby nemohl shodit hlavni smycku na konfiguracni chybe.
2. Upravit `on_connect` v `mqtt_to_postgres.py` pro kompatibilitu s `paho-mqtt` 2.x.
3. Zpevnit `connect_mqtt()` proti nedefinovanemu `client`.
4. Udelat WDT feed ve `flush_buffer()` casteji.
5. Sjednotit `README.md`, `AUDIT.md` a `postgres-push/README_mqtt_to_postgres.md` s aktualni implementaci.
