# tlak-pec

MicroPython senzorový systém pro Raspberry Pi Pico 2W. Podporuje **až 4 heterogenní senzory** na ADS1115 — každý senzor může mít vlastní převodní logiku (4-20 mA proudová smyčka, Pirani vakuometr, termočlánky, atd.). Zobrazuje hodnoty na 4-místném LED displeji a publikuje na MQTT broker.

## Hardware

### Komponenty

- **Raspberry Pi Pico 2W** — mikrokontrolér s WiFi
- **ADS1115** — 16-bit ADC převodník (I2C), 4 kanály
- **TM1637** — 4-místný 7-segmentový LED displej s desetinnými tečkami
- **Bočník** — rezistor pro převod proudu na napětí (pro 4-20 mA senzory)
- **Senzory** — průmyslové senzory s výstupem 4-20 mA, Pirani vakuometry, nebo jiné
- **Tlačítko** (volitelné) — přepínání zobrazeného senzoru na displeji
- **LED indikátory** (volitelné) — jedna LED na senzor, svítí při aktivním zobrazení

### Zapojení (příklad se dvěma senzory)

```
Senzor 1 (4-20 mA)              Senzor 2 (Pirani, 0-10V)
    │                               │
    ├── bočník ~100 Ohm ──→ GND     ├── napěťový dělič ──→ GND
    │       │                       │         │
    │       └── ADS1115 CH0         │         └── ADS1115 CH1
    │
ADS1115 I2C:
    SDA ── GPIO 26
    SCL ── GPIO 27

TM1637 displej:
    CLK ── GPIO 3
    DIO ── GPIO 2

Tlačítko (volitelné):
    GPIO 14 ── tlačítko ── GND
    (interní pull-up, stisk = sestupná hrana)

LED indikátory (volitelné, jedna na senzor):
    GPIO 15 ── 330Ω ── LED ── GND   (senzor 0)
    GPIO 16 ── 330Ω ── LED ── GND   (senzor 1)
```

### Princip měření

**Proudová smyčka 4-20 mA:** Proud prochází bočníkem, na kterém vzniká úbytek napětí:
- 4 mA × 100 Ohm ≈ 0.400 V (minimální hodnota)
- 20 mA × 100 Ohm ≈ 2.000 V (maximální hodnota)

**Pirani vakuometr:** Výstup snímače je typicky 0-10 V, ale ADC měří v omezeném rozsahu. Proto je nutný napěťový dělič. Naměřené napětí `u_meas` [V] za děličem se nejprve přepočítá: `u_actual = u_meas × u_divider`, kde `u_divider` je konfigurabilní koeficient děliče. Poté se tlak počítá regresním modelem: `p = exp(a + b·u_actual + c·√u_actual)` [mbar], kde koeficienty a, b, c jsou fitovány na hodnoty `u_actual` (tj. musí odpovídat zvolené hodnotě `u_divider`).

Každý senzor má vlastní `convert_raw()` metodu pro převod ADC hodnoty na výstupní veličinu.

### Rozlišení ADC a přesnost měření

ADS1115 je 16-bit signed ADC. Pro single-ended měření (kladná napětí) je využito 15 bitů (0–32767 kroků).

**Příklad pro 4-20 mA senzor s rozsahem 0–120 kPa:**

| Parametr | Hodnota |
|----------|---------|
| Gain | 4 (±1.024 V) |
| LSB (nejmenší krok) | 1.024 V / 32768 = 31.25 µV |
| Napěťový rozsah | 0.2–1.0 V (0.8 V span) |
| Kroků v rozsahu | 0.8 V / 31.25 µV ≈ 25 600 |
| Rozlišení tlaku | 120 kPa / 25 600 ≈ **0.005 kPa** |

Rozlišení ADC (~0.005 kPa) je horší než zobrazovaná přesnost (0.001 kPa na 3 des. místa). Poslední 1–2 desetinná místa na displeji proto nemají reálný význam — slouží pouze pro vizuální stabilitu zobrazení.

**Limitujícím faktorem je přesnost samotného senzoru** — průmyslové tlakové senzory mají typicky chybu 0.25–0.5 % z rozsahu (tj. 0.3–0.6 kPa pro 120 kPa senzor), což je ještě horší než rozlišení ADC.

## Architektura

```
+-------------------+
|   SensorManager   |  Hlavní smyčka, WDT, displej
+-------------------+
         |
         v
+-------------------+
|  SharedResources  |  WiFi, MQTT, I2C/ADC, Display (jedna instance)
+-------------------+
         ^
         | používá
+--------+-----------+------------------+
|                    |                  |
v                    v                  v
CurrentLoopSensor    PiraniSensor      (vlastní senzory...)
   (4-20mA)          (Pirani gauge)
         \            /
          \          /
           v        v
      +----------------+
      | SensorChannel  |  Základní třída (abstraktní convert_raw)
      +----------------+
```

## Závislosti

MicroPython knihovny, které musí být nahrané na Pico:

| Knihovna | Popis | Zdroj |
|----------|-------|-------|
| `ads1x15` | ADS1115 ADC driver | [robert-hh/ads1x15](https://github.com/robert-hh/ads1x15) |
| `tm1637` | TM1637 LED display driver | [mcauser/micropython-tm1637](https://github.com/mcauser/micropython-tm1637) |
| `umqtt.simple` | MQTT klient | Součást MicroPython firmware |

Z knihovny `tm1637` se používá podtřída `TM1637Decimal`, která podporuje individuální desetinné tečky za každým digitem.

## Instalace

### 1. MicroPython firmware

1. Stáhněte nejnovější MicroPython firmware pro **Pico 2W** z:
   https://micropython.org/download/RPI_PICO2_W/

   Soubor má název např. `RPI_PICO2_W-20241025-v1.24.0.uf2`

2. Připojte Pico 2W k počítači **se stisknutým tlačítkem BOOTSEL**

3. Pico se připojí jako USB mass storage zařízení (např. `RPI-RP2`)

4. Zkopírujte `.uf2` soubor na toto zařízení:
   ```bash
   cp RPI_PICO2_W-*.uf2 /media/$USER/RPI-RP2/
   ```

5. Pico se automaticky restartuje s MicroPythonem

### 2. Nástroj mpremote

Nainstalujte `mpremote` pro komunikaci s Pico:

```bash
pip install mpremote
```

Ověřte připojení:

```bash
mpremote connect /dev/ttyACM0 repl
# Ctrl+X pro ukončení REPL
```

### 3. Knihovny

Stáhněte a nahrajte požadované knihovny:

```bash
# ADS1115 ADC driver
wget https://raw.githubusercontent.com/robert-hh/ads1x15/master/ads1x15.py
mpremote connect /dev/ttyACM0 cp ads1x15.py :ads1x15.py

# TM1637 display driver
wget https://raw.githubusercontent.com/mcauser/micropython-tm1637/master/tm1637.py
mpremote connect /dev/ttyACM0 cp tm1637.py :tm1637.py
```

Alternativně pomocí `mip` (MicroPython package manager):

```bash
mpremote connect /dev/ttyACM0 mip install github:robert-hh/ads1x15
mpremote connect /dev/ttyACM0 mip install github:mcauser/micropython-tm1637
```

### 4. Konfigurace

Vytvořte `config.py` z šablony a vyplňte WiFi credentials, IP adresu MQTT brokeru a konfiguraci senzorů:

```bash
cp config.py.example config.py
# Upravte config.py s vašimi údaji
```

### 5. Nahrání aplikace

```bash
mpremote connect /dev/ttyACM0 cp config.py :config.py
mpremote connect /dev/ttyACM0 cp main.py :main.py
```

Nebo vše najednou:

```bash
mpremote connect /dev/ttyACM0 cp config.py :config.py + cp main.py :main.py + reset
```

Po restartu Pico se `main.py` spustí automaticky.

### 6. Ověření

Připojte se k REPL a sledujte výstup:

```bash
mpremote connect /dev/ttyACM0 repl
```

Měli byste vidět:
```
WiFi OK: 192.168.1.xx
```

Pro restart zařízení:
```bash
mpremote connect /dev/ttyACM0 reset
```

## Konfigurace

Veškeré parametry jsou v souboru `config.py`.

### WiFi

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `WIFI_SSID` | `""` | Název WiFi sítě |
| `WIFI_PASSWORD` | `""` | Heslo WiFi sítě |

### MQTT

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `MQTT_BROKER` | `"10.10.0.43"` | IP adresa MQTT brokeru |
| `MQTT_PORT` | `1883` | Port MQTT brokeru |
| `MQTT_CLIENT_ID` | `"pico_L200h"` | Identifikátor klienta |
| `MQTT_KEEPALIVE` | `60` | Keepalive interval v sekundách |
| `MQTT_STATUS_TOPIC` | `"sensor/L200h/status"` | Topic pro LWT a stav zařízení (online/offline) |
| `MQTT_USER` | `None` | MQTT username (`None` = anonymní přístup) |
| `MQTT_PASSWORD` | `None` | MQTT heslo |

### Piny

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `I2C_ID` | `1` | I2C sběrnice (0 nebo 1) |
| `I2C_SDA` | `26` | GPIO pin pro SDA |
| `I2C_SCL` | `27` | GPIO pin pro SCL |
| `I2C_FREQ` | `400000` | Frekvence I2C (Hz) |
| `TM_CLK` | `3` | GPIO pin pro CLK displeje |
| `TM_DIO` | `2` | GPIO pin pro DIO displeje |

### ADC (sdílené)

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `ADC_ADDRESS` | `0x48` | I2C adresa ADS1115 (viz tabulka níže) |
| `ADC_GAIN` | `2` | Gain ADS1115 (viz tabulka níže) |
| `ADC_RATE` | `3` | Rychlost vzorkování (viz tabulka níže) |
| `ADC_SAMPLES` | `5` | Počet vzorků pro průměrování |

**Adresa ADS1115** (podle zapojení ADDR pinu):

| ADDR pin | Adresa |
|----------|--------|
| GND | 0x48 |
| VDD | 0x49 |
| SDA | 0x4A |
| SCL | 0x4B |

Při inicializaci se provede I2C scan a ověří se přítomnost zařízení na dané adrese.

**Gain ADS1115:**

| Hodnota | Rozsah |
|---------|--------|
| 1 | ±4.096 V |
| 2 | ±2.048 V |
| 4 | ±1.024 V |
| 8 | ±0.512 V |
| 16 | ±0.256 V |

**Rate ADS1115 (rychlost vzorkování):**

| Hodnota | SPS |
|---------|-----|
| 0 | 8 |
| 1 | 16 |
| 2 | 32 |
| 3 | 64 |
| 4 | 128 |
| 5 | 250 |
| 6 | 475 |
| 7 | 860 |

### Displej

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `DISPLAY_SENSOR` | `0` | Index senzoru pro zobrazení (0 = první) |
| `BUTTON_PIN` | `None` | GPIO pin pro přepínací tlačítko (`None` = deaktivováno) |

### Systém

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `INTERVAL_S` | `1` | Interval měření v sekundách |
| `WDT_TIMEOUT_MS` | `8000` | Timeout watchdogu v ms |
| `PUBLISH_BUFFER_MAX` | `200` | Max. počet hodnot v zásobníku pro případ výpadku WiFi/MQTT |
| `DIAG_INTERVAL` | `60` | Publikovat diagnostiku každých N cyklů (0 = vypnuto) |
| `DIAG_TOPIC` | `"diag/L200h"` | MQTT topic pro diagnostiku |

### Senzory (SENSORS)

Seznam senzorů se konfiguruje v poli `SENSORS`. Společné parametry:

| Klíč | Povinný | Popis |
|------|---------|-------|
| `type` | ano | Typ senzoru: `"current_loop"`, `"pirani"`, nebo vlastní |
| `channel` | ano | ADC kanál (0-3) |
| `topic` | ano | MQTT topic pro publikaci hodnoty |
| `name` | ne | Název pro debug (výchozí: `"CH{channel}"`) |
| `status_topic` | ne | MQTT topic pro stav senzoru (výchozí: `{topic}/status`) |
| `precision` | ne | Počet desetinných míst pro MQTT (výchozí: 3) |
| `led_pin` | ne | GPIO pin pro LED indikátor aktivního senzoru (`None` = bez LED) |

**Parametry pro `current_loop`:**

| Klíč | Výchozí | Popis |
|------|---------|-------|
| `r_bocnik` | - | Odpor bočníku v Ohm |
| `i_min` | `0.004` | Minimální proud (A) |
| `i_max` | `0.020` | Maximální proud (A) |
| `p_min` | - | Minimální výstupní hodnota |
| `p_max` | - | Maximální výstupní hodnota |

**Parametry pro `pirani`:**

| Klíč | Výchozí | Popis |
|------|---------|-------|
| `a` | - | Regresní koeficient a |
| `b` | - | Regresní koeficient b |
| `c` | - | Regresní koeficient c |
| `u_divider` | `1.0` | Koeficient děliče: `u_actual = u_meas × u_divider` |
| `u_min` | `0.0` | Minimální změřené napětí za děličem (V), pod = ERR_LO |
| `u_max` | `v_ref` | Maximální změřené napětí za děličem (V), nad = ERR_HI |
| `p_min` | `1e-4` | Minimální tlak (mbar), pod = ERR_LO |
| `p_max` | `1000.0` | Maximální tlak (mbar), nad = ERR_HI |

> Regresní koeficienty musí být fitovány na hodnoty `u_actual` (tj. na napětí po přepočtu `u_divider`), ne na surové napětí z ADC.

#### Příklad konfigurace

```python
SENSORS = [
    {
        "type": "current_loop",
        "channel": 0,
        "topic": "sensor/L200h/p1",
        "name": "p1",
        "led_pin": 15,        # GPIO pin pro LED indikátor (None = bez LED)
        "r_bocnik": 99.1,     # [Ohm]
        "i_min": 0.004,       # [A]
        "i_max": 0.020,       # [A]
        "p_min": 0,           # [kPa]
        "p_max": 120,         # [kPa]
        "precision": 3,
    },
    {
        "type": "pirani",
        "channel": 1,
        "topic": "sensor/L200h/pirani",
        "led_pin": 16,        # GPIO pin pro LED indikátor (None = bez LED)
        "name": "Pirani",
        "a": -6.435,
        "b": 0.7418,
        "c": 0.8018,
        "u_divider": 5.009,   # koeficient děliče napětí
        "u_min": 0.0,         # [V] za děličem
        "u_max": 2.0,       # [V] za děličem
        "p_min": 1e-4,        # [mbar]
        "p_max": 1000.0,      # [mbar]
        "precision": 3,
    },
]
```

## Displej

4-místný 7-segmentový displej s desetinnými tečkami. Zobrazuje hodnotu ze senzoru vybraného parametrem `DISPLAY_SENSOR`. Tlačítkem na `BUTTON_PIN` lze cyklicky přepínat mezi senzory (sestupná hrana, debounce 200 ms); LED na `led_pin` aktivního senzoru svítí.

### Formátování hodnot

| Rozsah | Formát | Příklad |
|--------|--------|---------|
| -99 až -10 | 1 desetinné místo | `-12.3` |
| -9.99 až -0.01 | 2 desetinná místa | `-1.23` |
| 0.000 až 9.999 | 3 desetinná místa | `3.141` |
| 10.00 až 99.99 | 2 desetinná místa | `23.50` |
| 100.0 až 999.9 | 1 desetinné místo | `105.3` |
| < -99 nebo >= 1000 | mimo rozsah | `----` |

### Chování při chybách

| Stav | Zobrazení |
|------|-----------|
| Normální provoz | Hodnota senzoru |
| Chyba senzoru | Chybový kód (E-Lo, E-Hi, atd.) |
| WiFi/MQTT offline | Střídá: hodnota (2s) ↔ chybový kód (0.5s) |

## MQTT

Každý senzor publikuje na **dvě témata**:

| Topic | Obsah | retain | Kdy |
|-------|-------|--------|-----|
| `topic` | Hodnota + timestamp UTC (ISO 8601 s `Z`), např. `"23.456 2026-04-18T12:00:00Z"` | True | Pouze při platné hodnotě |
| `status_topic` | Stav senzoru | False | Vždy |

Stav senzoru (`status_topic`) je vždy přítomen a nabývá hodnot:

| Hodnota | Význam |
|---------|--------|
| `"OK"` | Platná hodnota |
| `"sensor_low"` | Napětí/hodnota pod rozsahem (ERR_LO) |
| `"sensor_high"` | Napětí/hodnota nad rozsahem (ERR_HI) |
| `"adc_error"` | Chyba ADC komunikace |
| `"config_error"` | Neplatná konfigurace |

Navíc se na `MQTT_STATUS_TOPIC` (LWT, `retain=True`) publikuje stav zařízení: `"online"` při startu, `"offline"` při neočekávaném odpojení.

### Autentizace

MQTT broker by měl být nakonfigurován s autentizací (username/password). Anonymní přístup se nedoporučuje pro produkční nasazení.

**Nastavení brokeru (Mosquitto):**

1. Vytvořte soubor s uživateli a hesly:
   ```bash
   # Vytvoření nového souboru s prvním uživatelem
   mosquitto_passwd -c /etc/mosquitto/passwd sensor
   # Přidání dalšího uživatele (bez -c, aby se soubor nepřepsal)
   mosquitto_passwd /etc/mosquitto/passwd dalsi_uzivatel
   ```

2. V `/etc/mosquitto/mosquitto.conf` nastavte:
   ```
   allow_anonymous false
   password_file /etc/mosquitto/passwd
   ```

3. Restartujte broker:
   ```bash
   systemctl restart mosquitto
   ```

**Nastavení klientů:**

- **Pico (config.py):** nastavte `MQTT_USER` a `MQTT_PASSWORD`
- **mqtt_to_postgres (mqtt_push.cfg):** odkomentujte `username` a `password` v sekci `[mqtt]`
- **Shell skripty (mqtt_show.sh):** obsahují flagy `-u` a `-P` — upravte credentials

### Diagnostika

Každých `DIAG_INTERVAL` měřicích cyklů se na `DIAG_TOPIC` publikuje JSON s diagnostikou zařízení (`retain=True`):

```json
{"uptime":3600,"rssi":-55,"mem":45000,"buf":3,"reconn_wifi":1,"reconn_mqtt":2,"ver":"1.4.0"}
```

| Klíč | Popis |
|------|-------|
| `uptime` | Doba běhu v sekundách |
| `rssi` | WiFi síla signálu (dBm), pouze při připojení |
| `mem` | Volná paměť (bajty) |
| `buf` | Počet zpráv v publish zásobníku |
| `reconn_wifi` | Počet WiFi reconnectů od startu |
| `reconn_mqtt` | Počet MQTT reconnectů od startu |
| `ver` | Verze firmware |

Timestamp v hodnotovém topicu je v **UTC** s příponou `Z` (ISO 8601, např. `2026-04-18T12:00:00Z`). Postgres/Grafana zobrazuje čas v lokální zóně podle session TZ — žádná DST logika na Picu není potřeba.

## Chybové kódy

| Kód | Význam | Zobrazí na displeji | Publikuje na MQTT |
|-----|--------|---------------------|-------------------|
| `E--1` | Selhání WiFi připojení | Ano | Ne |
| `E--2` | Selhání MQTT připojení | Ano | Ne |
| `E-Lo` | Napětí/hodnota pod rozsahem (odpojený senzor) | Ano | Ano (`sensor_low`) |
| `E-Hi` | Napětí/hodnota nad rozsahem (zkrat/porucha) | Ano | Ano (`sensor_high`) |
| `E--3` | Chyba ADC (I2C komunikace) | Ano | Ano (`adc_error`) |
| `E--4` | Neplatná konfigurace (ADC_GAIN, v_range=0) | Ano | Ano (`config_error`) |
| `E--5` | Fatální výjimka — zařízení čeká na WDT reset | Ano | Ne |
| `E--6` | MQTT autentizace selhala (špatné jméno/heslo) | Ano | Ne |

## Chování při chybách

Zařízení je navrženo pro nepřetržitý provoz. **Měření probíhá vždy**, i při výpadku WiFi nebo MQTT. Všechny subsystémy (displej, ADC, MQTT, WiFi) mohou selhat nezávisle — zařízení pokračuje s tím, co funguje.

### Recovery mechanismy

| Subsystém | Chování při selhání |
|-----------|---------------------|
| **Displej** | Zařízení běží bez displeje, měří a publikuje normálně |
| **ADC** | Automatická reinicializace s exponenciálním backoffem (1s → 60s) |
| **MQTT** | Automatický reconnect s exponenciálním backoffem (1s → 60s); periodický ping každých `MQTT_KEEPALIVE / 2` sekund s ověřením PINGRESP (2s timeout) — detekuje zombie TCP spojení; hodnoty naměřené během výpadku se ukládají do zásobníku (max `PUBLISH_BUFFER_MAX` položek) a po reconnectu se odešlou |
| **WiFi** | Non-blocking reconnect (deadline 90s), zařízení měří a zobrazuje i při výpadku; po obnovení WiFi se resetuje MQTT backoff a synchronizuje NTP |

### Watchdog

Hardwarový watchdog (výchozí timeout 8s) chrání proti zamrznutí. Všechna čekání v kódu krmí watchdog v intervalech `WDT_TIMEOUT_MS / 2`.

## Rozšíření o nový typ senzoru

Každý typ senzoru má vlastní `convert_raw()` metodu — žádná sdílená převodní logika není předpokládána:

1. Vytvořte třídu dědící z `SensorChannel`
2. Implementujte `__init__()` pro načtení parametrů z configu
3. Implementujte `convert_raw(self, raw)` vracející `(hodnota, chyba)` — libovolná logika
4. Zaregistrujte třídu v `SENSOR_TYPES`

```python
class ThermocoupleTypeK(SensorChannel):
    """Termočlánek typu K — nelineární převod"""

    def __init__(self, shared, cfg):
        super().__init__(shared, cfg)
        self.cold_junction = cfg.get("cold_junction", 25)

    def convert_raw(self, raw):
        voltage_uv = raw * self.shared.v_ref / 32767 * 1_000_000
        temp = self.cold_junction + voltage_uv * 0.025  # zjednodušeno
        if temp < -200 or temp > 1300:
            return None, ERR_HI
        return round(temp, 1), None

SENSOR_TYPES["thermocouple_k"] = ThermocoupleTypeK
```

## Diagnostika

Diagnostické výpisy na sériový port (UART / USB):

```
WiFi OK: 192.168.1.42
WiFi CHYBA
MQTT error: [Errno 113] ECONNABORTED
ADC init error: [Errno 5] EIO
Display init error: [Errno 5] EIO
Neznámý typ senzoru: invalid_type
Chybí klíč v konfiguraci senzoru: 'channel'
```

Připojte se k sériovému portu pro sledování:

```bash
mpremote connect /dev/ttyACM0 repl
```

## Linting

```bash
ruff check .
```

MicroPython moduly (`machine`, `umqtt`, `ads1x15`, `tm1637`, `network`) nejsou dostupné na desktopu — ignorujte chyby importů.
