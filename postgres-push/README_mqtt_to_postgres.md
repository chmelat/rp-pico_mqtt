# mqtt_to_postgres — MQTT → PostgreSQL daemon

Verze: `1.0.5`

Daemon čte senzorová data publikovaná přes MQTT a zapisuje je do PostgreSQL
databáze. Zpracovává všechny zprávy v platném formátu (`hodnota [timestamp]`)
na subscribovaných topicích. Timestamp je volitelný — bez něj se použije
aktuální čas serveru.

---

## Požadavky

- Python 3.9+  (kvůli `str.removeprefix`)
- PostgreSQL databáze s tabulkami `sensor` a `sensor_value` (viz níže)
- Spuštěný Mosquitto broker (nebo kompatibilní)

### Python závislosti

```bash
pip install paho-mqtt psycopg2-binary
```

README odpovida aktualnimu kodu, ktery je kompatibilni s `paho-mqtt` 1.x i 2.x.

---

## Databázové schéma

Daemon předpokládá existující schéma — nic nevytváří sám:

```sql
-- lookup tabulka senzorů
CREATE TABLE sensor (
    id   SERIAL PRIMARY KEY,
    name TEXT UNIQUE NOT NULL,
    ...
);

-- časová řada hodnot
CREATE TABLE sensor_value (
    sensor_id  INTEGER REFERENCES sensor(id),
    value      DOUBLE PRECISION,
    create_tms TIMESTAMPTZ
);
```

Jméno senzoru v DB (`sensor.name`) se odvozuje z MQTT topicu odebráním
prefixu (viz `strip_prefix` v konfiguraci):

```
MQTT topic:          sensor/L200h/pirani
strip_prefix:        sensor/
→ sensor.name v DB:  L200h/pirani
```

Před spuštěním daemona je nutné ručně vložit záznamy pro každý senzor:

```sql
INSERT INTO sensor (name) VALUES ('L200h/pirani');
INSERT INTO sensor (name) VALUES ('L200h/p1');
```

Obecně: pro každý MQTT topic ve tvaru `sensor/{device}/{sensor}` musí existovat
odpovídající řádek `{device}/{sensor}` v tabulce `sensor`. Pokud záznam chybí,
INSERT proběhne bez chyby, ale nevloží žádný řádek — hodnota se tiše ztratí.

---

## Konfigurace (`mqtt_push.cfg`)

```ini
[postgres]
# libpq connection string — stejný formát jako postgres.cfg
connection = dbname=zircodb user=sensor password=senSor7489 host=10.0.0.8 port=5432

[mqtt]
broker = 10.0.0.26
port   = 1883
topic  = sensor/#   # wildcard MQTT subscription

[sensor]
# Prefix odebraný z topicu při mapování na sensor.name v DB.
# Příklad: sensor/L200h/pirani → L200h/pirani
strip_prefix = sensor/
```

Soubor se předává jako poziční argument (výchozí: `mqtt_push.cfg` v aktuálním
adresáři).

---

## Spuštění

```bash
# z adresáře postgres-push/
python mqtt_to_postgres.py

# nebo s explicitní cestou ke konfiguraci
python mqtt_to_postgres.py /etc/sensors/mqtt_push.cfg
```

Zastavení: `Ctrl+C` nebo `SIGTERM` — daemon provede graceful shutdown (viz níže).

---

## Jak daemon funguje

### Filtrování MQTT zpráv

Daemon ignoruje:

| Topic pattern | Důvod ignorování |
|---|---|
| `…/status` | stavové zprávy (`online`, `offline`, `OK`, `sensor_low`) |
| payload neparsovatelný jako `float` | chybové řetězce, prázdné zprávy |

Zpracovává pouze zprávy, kde:
1. topic **nekončí** `/status`, a
2. první token payloadu lze převést na `float`.

Timestamp (vše za hodnotou) je volitelný. Pokud chybí, DB použije `now()`.
Podporované formáty timestampu: s oddělovačem `T` i s mezerou
(`2026-03-19T14:23:01`, `2026-03-19 14:23:01`).

Příklady z reálného provozu:

```
sensor/L200h/status online       → IGNOROVÁNO (končí /status)
sensor/L200h/pirani/status OK    → IGNOROVÁNO (končí /status)
sensor/L200h/pirani 0.152 2026-03-19T14:23:01  → ULOŽENO s timestamp z payloadu
sensor/L200h/pirani 0.152 2026-03-19 14:23:01  → ULOŽENO s timestamp z payloadu
sensor/L200h/p1 0.500                          → ULOŽENO s timestamp = now()
```

### Fronta

Každá přijatá hodnota se okamžitě vloží do in-memory fronty
(`collections.deque`, max 10 000 položek). Fronta odděluje MQTT vlákno od DB
vlákna — výpadek databáze nezpůsobí ztrátu dat (až do naplnění kapacity fronty).
Při přeplnění fronty se nejstarší položky zahazují (FIFO overflow).

### DB worker vlákno

Běží paralelně s MQTT smyčkou.

```
┌─────────────────────────────────────────────────┐
│ DB worker loop                                  │
│                                                 │
│  conn == None?                                  │
│    → psycopg2.connect()                         │
│    → při chybě: exponential backoff 1s → 60s   │
│                                                 │
│  fronta neprázdná?                              │
│    → vytáhnout batch (max 500 položek)          │
│    → executemany(INSERT…SELECT, batch)          │
│    → permanentní chyba (IntegrityError aj.):    │
│         → zahodit batch, rollback               │
│    → transientní chyba:                         │
│         → vrátit batch do fronty,               │
│           uzavřít conn, backoff                 │
│                                                 │
│  fronta prázdná?                                │
│    → spát 0.5 s                                 │
└─────────────────────────────────────────────────┘
```

Backoff se resetuje na 1 s po každém úspěšném připojení.

### SQL pattern

Vkládání hodnot je delegováno na PostgreSQL funkci s dvěma přetíženími:

```sql
-- s timestampem z payloadu
SELECT insert_sensor_value(%s, %s, %s)   -- (sensor_name, value, timestamp)

-- bez timestampu — DB použije now()
SELECT insert_sensor_value(%s, %s)       -- (sensor_name, value)
```

Parametry jsou předávány přes psycopg2 (`%s` placeholders) — SQL injection
nehrozí, escaping zajišťuje knihovna.

Formát payloadu: `hodnota [timestamp]`, například `0.152 2026-03-19T14:23:01`
nebo jen `0.152`. Timestamp může obsahovat oddělovač `T` i mezeru.

### Graceful shutdown

Na `SIGTERM` nebo `SIGINT` (Ctrl+C):

1. Odpojí MQTT klienta a zastaví jeho smyčku.
2. Nastaví `stop_event` — DB worker dopracuje zbývající frontu.
3. Čeká max 10 s na dokončení DB workeru.
4. Pokud ve frontě zůstaly nezapsané položky, zaloguje warning s jejich počtem.
5. Ukončí proces.

Data, která jsou ve frontě v okamžiku signálu, se tedy stihnou zapsat —
za předpokladu, že DB je dostupná a timeout 10 s je dostatečný.

---

## Provoz jako systemd service

Soubor `mqtt-to-postgres.service` v tomto adresáři je **system service** — běží
nezávisle na přihlášeném uživateli. Systemd vytvoří izolovaného dynamického
uživatele automaticky (`DynamicUser=yes`).

Skript se spouští z `/usr/local/bin`, konfigurační soubor se čte z `/usr/local/etc`.

> **Poznámka:** Daemon záměrně neběží pod rootem — přijímá data ze sítě a připojuje
> se k DB, takže root by zbytečně zvětšoval attack surface. `DynamicUser=yes` je
> pro tento účel doporučený přístup.

### Instalace souborů

```bash
sudo cp mqtt_to_postgres.py /usr/local/bin/
sudo cp mqtt_push.cfg /usr/local/etc/
sudo chmod 644 /usr/local/etc/mqtt_push.cfg   # čitelné pro dynamického uživatele
```

### Instalace service

```bash
sudo cp mqtt-to-postgres.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mqtt-to-postgres
```

### Správa

```bash
sudo systemctl status mqtt-to-postgres
sudo systemctl restart mqtt-to-postgres
sudo systemctl stop mqtt-to-postgres
```

### Logy přes journald

```bash
journalctl -u mqtt-to-postgres -f
```

---

## Ověření funkčnosti

```bash
# Simulovat MQTT zprávu
mosquitto_pub -h 10.0.0.26 -t sensor/L200h/pirani -m "0.152 2026-03-19T14:23:01"

# Ověřit v DB
psql "dbname=zircodb user=sensor password=senSor7489 host=10.0.0.8" -c "
  SELECT s.name, sv.value, sv.create_tms
  FROM sensor_value sv
  JOIN sensor s ON s.id = sv.sensor_id
  ORDER BY sv.create_tms DESC
  LIMIT 5;"
```

Očekávaný výstup:

```
      name      | value |          create_tms
----------------+-------+-------------------------------
 L200h/pirani   | 0.152 | 2026-03-19 14:23:01.843+00
 ...
```

---

## Logy

Daemon loguje na stdout ve formátu `YYYY-MM-DD HH:MM:SS LEVEL zpráva`:

```
2026-03-19 14:20:00 INFO Connected to PostgreSQL
2026-03-19 14:20:01 INFO MQTT connected, subscribing to sensor/#
2026-03-19 14:21:05 WARNING DB connect failed: ... — retry in 2s
2026-03-19 14:23:07 INFO Shutting down...
2026-03-19 14:23:07 INFO DB connection closed
```

Pro debug výpis počtu vložených řádků per batch:

```bash
python mqtt_to_postgres.py 2>&1 | grep -v DEBUG   # potlačit debug
# nebo spustit s upravenou úrovní:
```

Úroveň logování lze změnit editací `logging.basicConfig(level=logging.INFO, …)`
v hlavičce souboru (např. na `logging.DEBUG` pro detailní výpis).

---

## Omezení a known issues

- **Časová značka**: přebírá se z MQTT payloadu (vše za hodnotou). Pokud v payloadu
  chybí, použije se serverový čas (`now()`). Pokud zdroj posílá nepřesný čas,
  projeví se to přímo v `create_tms`.
- **Přeplnění fronty**: při výpadku DB delším než cca `10 000 × interval_měření`
  se nejstarší data začnou zahazovat. Kapacita (`QUEUE_MAX = 10_000`) je
  konfigurovatelná přímo v kódu.
- **Neznámý senzor**: chování při neexistujícím `sensor.name` závisí na
  implementaci funkce `insert_sensor_value()` na straně PostgreSQL.
- **Bez autentizace MQTT**: broker je předpokládán bez hesla. paho-mqtt podporuje
  `client.username_pw_set()` — přidejte do `[mqtt]` sekce a kódu dle potřeby.
