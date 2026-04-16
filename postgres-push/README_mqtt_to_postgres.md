# mqtt_to_postgres

Verze: `1.1.2`

Daemon, který poslouchá MQTT broker a ukládá přijatá senzorová data do
PostgreSQL. Běží na pozadí, sám se připojí k MQTT i k databázi a zapisuje
hodnoty průběžně.

```
  Senzor → MQTT broker → mqtt_to_postgres → PostgreSQL
```

---

## Formát MQTT zpráv

Daemon zpracovává zprávy ve tvaru:

```
hodnota [timestamp]
```

Timestamp je volitelný. Pokud chybí, použije se aktuální čas serveru (UTC)
v okamžiku přijetí MQTT zprávy, s přesností na mikrosekundy.

Příklady:

| MQTT topic | Payload | Co se stane |
|---|---|---|
| `sensor/L200h/pirani` | `0.152` | Uloží se s časem `now()` |
| `sensor/L200h/pirani` | `0.152 2026-03-19T14:23:01` | Uloží se s daným časem |
| `sensor/L200h/pirani` | `0.152 2026-03-19 14:23:01` | Totéž (mezera místo `T`) |
| `sensor/L200h/pirani` | `0.152 2026-03-19T14:23:01.274` | Timestamp se zlomky sekund |
| `sensor/L200h/pirani/status` | `OK` | Ignorováno (topic končí `/status`) |
| `sensor/L200h/status` | `online` | Ignorováno (topic končí `/status`) |

Ignorují se:
- Zprávy na topicích končících `/status` (stavové zprávy senzorů).
- Zprávy, kde payload není číslo.

---

## Instalace

### 1. Nainstaluj závislosti

```bash
pip install paho-mqtt psycopg2-binary
```

Vyžaduje Python 3.9+. Kompatibilní s `paho-mqtt` 1.x i 2.x.

### 2. Vytvoř konfigurační soubor

Zkopíruj šablonu a vyplň přístupové údaje:

```bash
cp mqtt_push.cfg.example mqtt_push.cfg
```

Obsah souboru `mqtt_push.cfg`:

```ini
[postgres]
connection = dbname=zircodb user=sensor password=heslo host=10.0.0.8 port=5432

[mqtt]
broker = 10.0.0.26
port   = 1883
topic  = sensor/#
username = sensor
password = mojeheslo

[sensor]
strip_prefix = sensor/
```

- **`connection`** — připojení k PostgreSQL (formát libpq).
- **`topic`** — MQTT topic k odběru. `sensor/#` znamená "vše začínající na `sensor/`".
- **`username`** / **`password`** — přihlašovací údaje k MQTT brokeru. Vynechte nebo zakomentujte pro anonymní přístup.
- **`strip_prefix`** — co oříznout z topicu, aby vzniklo jméno senzoru v DB.
  Například: topic `sensor/L200h/pirani` → po oříznutí `sensor/` → v DB se hledá
  senzor s názvem `L200h/pirani`.

### 3. Spusť ručně (pro test)

```bash
python mqtt_to_postgres.py mqtt_push.cfg
```

Bez argumentu hledá `mqtt_push.cfg` v aktuálním adresáři. Ukončení: `Ctrl+C`.

### 4. Zkopíruj soubory pro systemd

```bash
sudo cp mqtt_to_postgres.py /usr/local/bin/
sudo cp mqtt_push.cfg       /usr/local/etc/
sudo chmod 644 /usr/local/etc/mqtt_push.cfg
```

### 5. Nainstaluj a spusť service

```bash
sudo cp mqtt-to-postgres.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mqtt-to-postgres
```

### 6. Kontrola

```bash
sudo systemctl status mqtt-to-postgres    # stav
journalctl -u mqtt-to-postgres -f          # živé logy
```

Další příkazy:

```bash
sudo systemctl restart mqtt-to-postgres    # restart
sudo systemctl stop mqtt-to-postgres       # zastavení
```

> **Poznámka:** Service používá `DynamicUser=yes` — neběží pod rootem, systemd
> pro něj automaticky vytvoří izolovaného uživatele.

---

## Jak to funguje uvnitř

### Architektura

Daemon má dvě vlákna:

1. **MQTT vlákno** — přijímá zprávy z brokeru a řadí je do fronty.
2. **DB worker vlákno** — vybírá zprávy z fronty a zapisuje je do PostgreSQL.

```
MQTT broker
    │
    ▼
MQTT vlákno  ──→  fronta (max 10 000 položek)  ──→  DB worker  ──→  PostgreSQL
```

Oddělení přes frontu znamená, že výpadek databáze nezpůsobí ztrátu dat —
zprávy se hromadí ve frontě, dokud se DB nevrátí. Teprve při naplnění
fronty (10 000 položek) se nejstarší data začnou zahazovat.

### Odolnost proti výpadkům

- Při výpadku DB se worker pokouší znovu připojit s exponenciálním čekáním
  (1 s → 2 s → 4 s → … → max 60 s). Po úspěšném připojení se čekání resetuje.
- Trvalé chyby (např. neexistující senzor v DB) způsobí zahození daného batche
  — neopakují se donekonečna.
- MQTT klient se při ztrátě spojení s brokerem připojí automaticky sám
  (vlastnost knihovny paho-mqtt).

### Ukončení (graceful shutdown)

Na `Ctrl+C` nebo `SIGTERM`:

1. Odpojí se od MQTT brokeru.
2. DB worker dopíše zbývající frontu (max 10 s).
3. Pokud ve frontě zbyly nezapsané položky, zaloguje varování.

### Logy

Formát: `YYYY-MM-DD HH:MM:SS LEVEL zpráva`. Příklady:

```
2026-03-19 14:20:00 INFO Connected to PostgreSQL
2026-03-19 14:20:01 INFO MQTT connected, subscribing to sensor/#
2026-03-19 14:21:05 WARNING DB connect failed: ... — retry in 2s
```

Úroveň logování lze změnit v hlavičce `mqtt_to_postgres.py`
(`logging.INFO` → `logging.DEBUG`).

---

## Příloha: Databázová funkce

Daemon nic v DB nevytváří — předpokládá existující funkci `insert_sensor_value`,
kterou volá ve dvou variantách:

```sql
-- s timestampem
SELECT insert_sensor_value('L200h/pirani', 0.152, '2026-03-19 14:23:01');

-- bez timestampu (použije now())
SELECT insert_sensor_value('L200h/pirani', 0.152);
```

Pro každý MQTT topic musí existovat odpovídající senzor v databázi (jméno =
topic bez prefixu). Pokud senzor chybí, funkce vyhodí výjimku a batch se zahodí.

Definice funkcí jsou v souboru `sensor_value_functions.sql`.

---

## Omezení

- **Fronta**: při výpadku DB delším než `10 000 × interval měření` se začnou
  zahazovat nejstarší data. Kapacita je nastavitelná v kódu (`QUEUE_MAX`).
- **MQTT autentizace**: při špatných credentials (CONNACK rc 4/5) se daemon
  ukončí s chybovou hláškou — neopakuje připojení, dokud se neopraví konfigurace.
- **Nepřesný čas z payloadu**: pokud senzor posílá špatný timestamp, projeví se
  přímo v `create_tms` v DB.
