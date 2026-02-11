# tlak-pec

MicroPython senzorový systém pro Raspberry Pi Pico 2W. Podporuje **až 4 heterogenní senzory** na ADS1115 — každý senzor může mít vlastní převodní logiku (4-20 mA proudová smyčka, Pirani vakuometr, termočlánky, atd.). Zobrazuje hodnoty na 4-místném LED displeji a publikuje na MQTT broker.

## Hardware

### Komponenty

- **Raspberry Pi Pico 2W** — mikrokontrolér s WiFi
- **ADS1115** — 16-bit ADC převodník (I2C), 4 kanály
- **TM1637** — 4-místný 7-segmentový LED displej s desetinnými tečkami
- **Bočník** — rezistor pro převod proudu na napětí (pro 4-20 mA senzory)
- **Senzory** — průmyslové senzory s výstupem 4-20 mA, Pirani vakuometry, nebo jiné

### Zapojení (příklad se dvěma senzory)

```
Senzor 1 (4-20 mA)              Senzor 2 (Pirani, 0-10V)
    │                               │
    ├── bočník 50 Ohm ──→ GND       ├── dělič 1:10 ──→ GND
    │       │                       │       │
    │       └── ADS1115 CH0         │       └── ADS1115 CH1
    │                               │
    │                          (0-10V → 0-1V)
    │
ADS1115 I2C:
    SDA ── GPIO 0
    SCL ── GPIO 1

TM1637 displej:
    CLK ── GPIO 16
    DIO ── GPIO 17
```

### Princip měření

**Proudová smyčka 4-20 mA:** Proud prochází bočníkem, na kterém vzniká úbytek napětí:
- 4 mA × 50 Ohm = 0.200 V (minimální hodnota)
- 20 mA × 50 Ohm = 1.000 V (maximální hodnota)

**Pirani vakuometr:** Výstup snímače je 0-10 V, ale ADC měří v rozsahu 0-1 V (gain=4, ±1.024 V). Proto je nutný napěťový dělič 1:10 (např. 9 kΩ + 1 kΩ). Naměřené napětí u [V] na ADC se převádí na tlak regresním modelem: `p = exp(a + b·u + c·√u)` [mbar], kde koeficienty a, b, c jsou fitovány na dělenou charakteristiku (tj. u = U_pirani / 10).

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

Upravte `config.py` — vyplňte WiFi credentials, IP adresu MQTT brokeru a konfiguraci senzorů:

```bash
cp config.py config.py.local
# Upravte config.py.local s vašimi údaji
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
| `WIFI_SSID` | `"tvoje_SSID"` | Název WiFi sítě |
| `WIFI_PASSWORD` | `"tvoje_heslo"` | Heslo WiFi sítě |

### MQTT

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `MQTT_BROKER` | `"192.168.1.x"` | IP adresa MQTT brokeru |
| `MQTT_PORT` | `1883` | Port MQTT brokeru |
| `MQTT_CLIENT_ID` | `"pico_tlak"` | Identifikátor klienta |

### Piny

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `I2C_ID` | `0` | I2C sběrnice |
| `I2C_SDA` | `0` | GPIO pin pro SDA |
| `I2C_SCL` | `1` | GPIO pin pro SCL |
| `I2C_FREQ` | `400000` | Frekvence I2C (Hz) |
| `TM_CLK` | `16` | GPIO pin pro CLK displeje |
| `TM_DIO` | `17` | GPIO pin pro DIO displeje |

### ADC (sdílené)

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `ADC_ADDRESS` | `0x48` | I2C adresa ADS1115 (viz tabulka níže) |
| `ADC_GAIN` | `4` | Gain ADS1115 (viz tabulka níže) |
| `ADC_RATE` | `0` | Rychlost vzorkování (viz tabulka níže) |

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

### Systém

| Parametr | Výchozí | Popis |
|----------|---------|-------|
| `INTERVAL_S` | `1` | Interval měření v sekundách |
| `WDT_TIMEOUT_MS` | `8000` | Timeout watchdogu v ms |

### Senzory (SENSORS)

Seznam senzorů se konfiguruje v poli `SENSORS`. Společné parametry:

| Klíč | Popis |
|------|-------|
| `type` | Typ senzoru: `"current_loop"`, `"pirani"`, nebo vlastní |
| `channel` | ADC kanál (0-3) |
| `topic` | MQTT topic pro publikaci |
| `name` | Název pro debug (volitelný) |

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
| `u_min` | `0.0` | Minimální napětí (V), pod = ERR_LO |
| `u_max` | `v_ref` | Maximální napětí (V), nad = ERR_HI |
| `p_min` | `1e-4` | Minimální tlak (mbar), pod = ERR_LO |
| `p_max` | `1000.0` | Maximální tlak (mbar), nad = ERR_HI |

#### Příklad konfigurace

```python
SENSORS = [
    {
        "type": "current_loop",
        "channel": 0,
        "topic": "sensor/tlak/pec1",
        "name": "Pec 1",
        "r_bocnik": 50,
        "i_min": 0.004,
        "i_max": 0.020,
        "p_min": 0,
        "p_max": 120,
    },
    {
        "type": "pirani",
        "channel": 1,
        "topic": "sensor/L200h/pirani",
        "name": "Pirani",
        "a": -6.435,
        "b": 7.418,
        "c": 2.536,
        "p_min": 1e-4,
        "p_max": 1000.0,
    },
]
```

## Displej

4-místný 7-segmentový displej s desetinnými tečkami. Zobrazuje hodnotu ze senzoru vybraného parametrem `DISPLAY_SENSOR`.

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

Hodnoty se publikují na topic definovaný pro každý senzor s příznakem `retain=True`.

### Formát zpráv

- **Platná hodnota:** číslo jako string, např. `"23.456"` (3 desetinná místa)
- **Chybový stav:** chybový kód jako string, např. `"E-Lo"`

Subscriber rozliší platnou hodnotu od chyby podle formátu — číslo = platná hodnota, `E-*` = chyba.

## Chybové kódy

| Kód | Význam | Publikuje na MQTT |
|-----|--------|-------------------|
| `E--1` | Selhání WiFi připojení | Ne |
| `E--2` | Selhání MQTT připojení | Ne |
| `E-Lo` | Napětí/hodnota pod rozsahem (odpojený senzor) | Ano |
| `E-Hi` | Napětí/hodnota nad rozsahem (zkrat/porucha) | Ano |
| `E--3` | Chyba ADC (I2C komunikace) | Ano |
| `E--4` | Neplatná konfigurace (ADC_GAIN, v_range=0) | Ano |

## Chování při chybách

Zařízení je navrženo pro nepřetržitý provoz. **Měření probíhá vždy**, i při výpadku WiFi nebo MQTT. Všechny subsystémy (displej, ADC, MQTT, WiFi) mohou selhat nezávisle — zařízení pokračuje s tím, co funguje.

### Recovery mechanismy

| Subsystém | Chování při selhání |
|-----------|---------------------|
| **Displej** | Zařízení běží bez displeje, měří a publikuje normálně |
| **ADC** | Automatická reinicializace s exponenciálním backoffem (1s → 60s) |
| **MQTT** | Automatický reconnect s exponenciálním backoffem (1s → 60s), socket se korektně uvolňuje |
| **WiFi** | Automatický reconnect při každém měřicím cyklu |

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
