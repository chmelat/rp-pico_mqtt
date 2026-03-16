# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MicroPython sensor system for Raspberry Pi Pico 2W. Reads up to 4 heterogeneous sensors via ADS1115 ADC (I2C), displays values on a TM1637 4-digit LED display, and publishes to an MQTT broker over WiFi.

**Target runtime: MicroPython on RP2350 (Pico 2W)** — standard CPython modules are not available. MicroPython-specific modules (`machine`, `network`, `umqtt`, `time.ticks_ms`, etc.) will cause import errors on desktop.

## Commands

### Linting
```bash
ruff check .
```
Ignore import errors for MicroPython-only modules (`machine`, `umqtt`, `ads1x15`, `tm1637`, `network`).

### Deploy to device
```bash
# Copy both files and reset
mpremote connect /dev/ttyACM0 cp config.py :config.py + cp main.py :main.py + reset

# Monitor serial output
mpremote connect /dev/ttyACM0 repl
```

### Monitor MQTT
```bash
mosquitto_sub -h 10.10.0.43 -t "#" -v
```

### Configuration setup
```bash
cp config.py.example config.py
# Edit config.py with WiFi credentials, MQTT broker, and sensor parameters
```

## Architecture

All code lives in two files:

- **`main.py`** — all application logic
- **`config.py`** — all user-configurable parameters (WiFi, MQTT, pins, sensors list)

### Class hierarchy in `main.py`

```
SensorManager          # main loop, WDT, display routing
  └─ SharedResources   # WiFi, MQTT, I2C/ADC, display — one instance shared by all sensors
       └─ used by all SensorChannel instances

SensorChannel          # abstract base: read() → convert_raw() → publish()
  ├─ CurrentLoopSensor # 4-20 mA linear conversion
  └─ PiraniSensor      # vacuum gauge: p = exp(a + b·u + c·√u)
```

`SENSOR_TYPES` dict maps config `"type"` strings to classes. `create_sensor()` is the factory.

### Key design patterns

**Resilience:** All subsystems (display, ADC, WiFi, MQTT) fail independently. Measurement always runs. ADC and MQTT use exponential backoff for reconnection (1s → 60s cap).

**WDT safety:** `safe_sleep_ms()` feeds the hardware watchdog every `WDT_TIMEOUT_MS // 2` ms. All long waits must use this method. MQTT broker TCP pre-test uses a 4s socket timeout to prevent WDT starvation.

**MQTT:** Uses `umqtt.simple`. LWT publishes `"offline"` on unexpected disconnect; `"online"` on connect. Each sensor publishes to `{topic}` (value, only when valid) and `{topic}/status` (`"OK"` or error string), both `retain=True`.

**Display:** Shows sensor selected by `DISPLAY_SENSOR` index. When WiFi/MQTT fails but sensor reads are valid, alternates: value (2s) / error code (0.5s).

### Adding a new sensor type

1. Subclass `SensorChannel`
2. In `__init__`, call `super().__init__(shared, cfg)` and read type-specific config keys
3. Implement `convert_raw(self, raw) -> (value, error)` where `raw` is a 15-bit signed ADC integer; return `(None, ERR_*)` on error
4. Register in `SENSOR_TYPES`

### Error codes

| Code | Meaning |
|------|---------|
| `E--1` | WiFi failed |
| `E--2` | MQTT failed |
| `E-Lo` | Sensor below range |
| `E-Hi` | Sensor above range |
| `E--3` | ADC I2C error |
| `E--4` | Config error (bad gain / zero range) |
| `E--5` | Fatal exception — waiting for WDT reset |
