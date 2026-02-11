# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MicroPython IoT sensor system for **Raspberry Pi Pico 2W**. Supports **up to 4 heterogeneous sensors** on ADS1115 ADC. Each sensor can have completely different conversion logic (4-20 mA current loop, Pirani gauge, thermocouples, etc.). Displays readings on a TM1637Decimal 4-digit LED display and publishes values over MQTT. Comments and error messages are in Czech.

## Deployment

No build system. Files are copied directly to the Pico 2W filesystem (typically via Thonny, mpremote, or rshell). The device runs `main.py` on boot automatically per MicroPython convention.

```bash
# Example using mpremote
mpremote connect /dev/ttyACM0 cp main.py :main.py
mpremote connect /dev/ttyACM0 cp config.py :config.py
```

## Linting

The project uses **ruff** and **mypy** (cache dirs present). Run against standard Python but note MicroPython-specific modules (`machine`, `umqtt`, `ads1x15`, `tm1637`, `network`) will not resolve on desktop.

## Architecture

Two files, multiple classes:

```
SensorManager          Main loop, WDT, display
       │
       v
SharedResources        WiFi, MQTT, I2C/ADC, Display (single instance)
       ^
       │ uses
       ├──────────────┐
       v              v
CurrentLoopSensor   PiraniSensor    (+ custom sensors)
    (4-20mA)        (Pirani gauge)
       └──────┬──────┘
              v
       SensorChannel   Base class (abstract convert_raw)
```

### config.py

All hardware pins, WiFi/MQTT credentials, ADC gain/rate, watchdog timeout, measurement interval. Sensors are configured in `SENSORS` list with per-sensor parameters:

```python
SENSORS = [
    {
        "type": "current_loop",  # or "pirani", or custom
        "channel": 0,            # ADC channel (0-3)
        "topic": "sensor/tlak/pec1",
        "name": "Pec 1",
        # type-specific params depend on sensor class
    },
]
```

`DISPLAY_SENSOR` selects which sensor to show on display (index into SENSORS).

### main.py

- **SharedResources**: Manages shared hardware (display, wlan, i2c, adc, mqtt, wdt)
  - `safe_sleep_ms()`: All sleeps feed watchdog in chunks of `WDT_TIMEOUT_MS // 2`
  - `_init_adc()`: I2C + ADS1115 initialization with exponential backoff (1s → 60s max), includes I2C scan to verify device presence
  - `read_adc_channel(channel)`: Returns `(raw, error)`, median filter over `ADC_SAMPLES` readings (use odd number), triggers re-init on `OSError`
  - `connect_wifi()`, `connect_mqtt()`: With exponential backoff
  - `publish(topic, value)`: Publishes to specified topic with retain
  - `show_value()`: Display with adaptive precision, shows "----" for out-of-range
  - `show_error()`: Display error code

- **SensorChannel**: Abstract base class for sensor channels
  - `read()`: Calls `read_adc_channel()` then `convert_raw()`, stores `last_value`/`last_error`
  - `convert_raw(raw)`: Abstract — each sensor type implements its own conversion logic
  - `publish()`: Publishes `last_value` or `last_error` to sensor's topic

- **CurrentLoopSensor(SensorChannel)**: 4-20 mA current loop sensor
  - Params: `r_bocnik`, `i_min`, `i_max`, `p_min`, `p_max`
  - Own `convert_raw()`: voltage from current × shunt, linear map to output value

- **PiraniSensor(SensorChannel)**: Pirani vacuum gauge
  - Params: `a`, `b`, `c` (regression coefficients), `u_min`, `u_max`, `p_min`, `p_max`
  - Own `convert_raw()`: `p = exp(a + b*u + c*sqrt(u))` [mbar]

- **SensorManager**: Main loop orchestrator
  - Creates sensors via `create_sensor()` factory (filters out failed sensors)
  - `_read_all()`: Reads all sensors
  - `_publish_all()`: Publishes all sensors, returns success status
  - `_update_display()`: Shows selected sensor, alternates value/error on connection failure
  - `run()`: Main loop with WDT, WiFi check, measurement interval

### Adding New Sensor Types

Each sensor type has its own `convert_raw()` — no shared conversion logic assumed:

1. Create class inheriting from `SensorChannel`
2. Implement `__init__()` to load config parameters
3. Implement `convert_raw(self, raw)` returning `(value, error)` — can be any conversion logic
4. Register in `SENSOR_TYPES` dict

```python
class ThermocoupleTypeK(SensorChannel):
    """Thermocouple with non-linear conversion"""
    def __init__(self, shared, cfg):
        super().__init__(shared, cfg)
        self.cold_junction = cfg.get("cold_junction", 25)

    def convert_raw(self, raw):
        voltage_uv = raw * self.shared.v_ref / 32767 * 1_000_000
        temp = self.cold_junction + voltage_uv * 0.025  # simplified
        if temp < -200 or temp > 1300:
            return None, ERR_HI
        return round(temp, 1), None

SENSOR_TYPES["thermocouple_k"] = ThermocoupleTypeK
```

## Timing

All timing uses `time.ticks_ms()` / `time.ticks_add()` / `time.ticks_diff()` consistently — never `time.time()` or `time.sleep()`. This avoids RTC dependency and handles tick wrap-around correctly.

## Hardware Wiring (from config.py defaults)

```
Sensor 1 (4-20mA) → 50Ω shunt → ADS1115 CH0
Sensor 2 (Pirani)  → ADS1115 CH1
...
ADS1115 I2C: SDA=GPIO0, SCL=GPIO1
TM1637Decimal display: CLK=GPIO16, DIO=GPIO17
```

## Error Codes (shown on display)

| Code | Meaning |
|------|---------|
| E--1 | WiFi connection failure |
| E--2 | MQTT broker connection failure |
| E-Lo | Sensor voltage or value too low (disconnected) |
| E-Hi | Sensor voltage or value too high (short/fault) |
| E--3 | ADC device error |
| E--4 | Invalid config (ADC_GAIN, or v_range=0) |

## Display Behavior

- Normal: shows sensor value with adaptive precision
- Sensor error: shows error code (E-Lo, E-Hi, etc.)
- WiFi/MQTT error: alternates value (2s) and error code (0.5s)
- Out of range (< -99 or >= 1000): shows "----"

## ADS1115 API (robert-hh/ads1x15)

Gain is set in the constructor: `ADS1115(i2c, gain=N)`. Channel and rate are per-read: `adc.read(rate, channel1[, channel2])`. Parameter order matters — rate first, then channel. If `channel2` is given, differential mode is used. Rate 0–7 maps to 8–860 SPS for ADS1115.

## MicroPython Dependencies

These libraries must be present on the Pico filesystem or in firmware:
- `ads1x15` — ADS1115 ADC driver ([robert-hh/ads1x15](https://github.com/robert-hh/ads1x15))
- `tm1637` — 7-segment LED display driver, uses `TM1637Decimal` subclass for per-digit decimal points ([mcauser/micropython-tm1637](https://github.com/mcauser/micropython-tm1637))
- `umqtt.simple` — Lightweight MQTT client (often bundled with firmware)
