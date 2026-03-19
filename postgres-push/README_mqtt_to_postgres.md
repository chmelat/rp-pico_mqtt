# mqtt_to_postgres — MQTT → PostgreSQL daemon

Daemon čte senzorová data publikovaná Raspberry Pi Pico 2W přes MQTT a zapisuje
je do PostgreSQL databáze. Navazuje na stávající C systém (DCON) a používá
identický SQL pattern (`INSERT … SELECT`) jako `postgres_query.c`.

---

## Požadavky

- Python 3.9+  (kvůli `str.removeprefix`)
- PostgreSQL databáze s tabulkami `sensor` a `sensor_value` (viz níže)
- Spuštěný Mosquitto broker (nebo kompatibilní)

### Python závislosti

```bash
pip install paho-mqtt psycopg2-binary
```

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
broker = 10.10.0.43
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
2. payload lze převést na `float`.

Příklady z reálného provozu:

```
sensor/L200h/status online       → IGNOROVÁNO (končí /status)
sensor/L200h/pirani/status OK    → IGNOROVÁNO (končí /status)
sensor/L200h/pirani 0.152        → ULOŽENO jako L200h/pirani = 0.152
sensor/L200h/p1 0.500            → ULOŽENO jako L200h/p1 = 0.500
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
│    → při chybě: vrátit batch do fronty,         │
│                 uzavřít conn, backoff           │
│                                                 │
│  fronta prázdná?                                │
│    → spát 0.5 s                                 │
└─────────────────────────────────────────────────┘
```

Backoff se resetuje na 1 s po každém úspěšném připojení.

### SQL pattern

Identický s `postgres_query.c` (řádky 224–227):

```sql
INSERT INTO sensor_value (sensor_id, value, create_tms)
SELECT id, %s, %s FROM sensor WHERE name = %s
```

Parametry jsou předávány přes psycopg2 (`%s` placeholders) — SQL injection
nehrozí, escaping zajišťuje knihovna.

Časová značka je generována v UTC na serveru daemona (`datetime.now(timezone.utc)`),
nikoliv převzata z MQTT zprávy.

### Graceful shutdown

Na `SIGTERM` nebo `SIGINT` (Ctrl+C):

1. Odpojí MQTT klienta a zastaví jeho smyčku.
2. Nastaví `stop_event` — DB worker dopracuje zbývající frontu.
3. Čeká max 10 s na dokončení DB workeru.
4. Ukončí proces.

Data, která jsou ve frontě v okamžiku signálu, se tedy stihnou zapsat —
za předpokladu, že DB je dostupná a timeout 10 s je dostatečný.

---

## Provoz jako systemd service

Soubor `mqtt-to-postgres.service` v tomto adresáři je připraven pro **user service**
(běží pod uživatelem, který jej nainstaluje — `User=` se neuvádí, cesta se odvozuje
přes `%h`).

### Instalace

```bash
mkdir -p ~/.config/systemd/user/
cp mqtt-to-postgres.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now mqtt-to-postgres
```

### Správa

```bash
systemctl --user status mqtt-to-postgres
systemctl --user restart mqtt-to-postgres
systemctl --user stop mqtt-to-postgres
```

### Logy přes journald

```bash
journalctl --user -u mqtt-to-postgres -f
```

---

## Ověření funkčnosti

```bash
# Simulovat MQTT zprávu
mosquitto_pub -h 10.10.0.43 -t sensor/L200h/pirani -m 0.152

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

Úroveň logování lze změnit editací `logging.basicConfig(level=logging.DEBUG, …)`
v hlavičce souboru.

---

## Omezení a known issues

- **Časová značka**: generuje se na straně daemona, ne Pica. Zpoždění způsobené
  výpadkem DB nebo plnou frontou se projeví rozdílem `create_tms` a skutečného
  času měření.
- **Přeplnění fronty**: při výpadku DB delším než cca `10 000 × interval_měření`
  se nejstarší data začnou zahazovat. Kapacita (`QUEUE_MAX = 10_000`) je
  konfigurovatelná přímo v kódu.
- **Neznámý senzor**: pokud `sensor.name` neexistuje v tabulce `sensor`, INSERT
  proběhne bez chyby, ale nevloží žádný řádek — hodnota se tiše ztratí. Viz
  sekce Databázové schéma.
- **Bez autentizace MQTT**: broker je předpokládán bez hesla. paho-mqtt podporuje
  `client.username_pw_set()` — přidejte do `[mqtt]` sekce a kódu dle potřeby.
