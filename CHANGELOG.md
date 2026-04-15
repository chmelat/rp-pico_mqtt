# Changelog

All notable changes to this project will be documented in this file.

## mqtt_to_postgres.py 1.1.0

### Added
- MQTT authentication: reads optional `username` and `password` from `[mqtt]` section
  in config file. If set, calls `username_pw_set()` before connecting.

## mqtt_to_postgres.py 1.0.3

### Fixed
- Permanent DB errors (duplicate key, sensor not found) now discard the failed batch instead
  of re-queuing it. Previously, a single bad item caused an infinite retry loop requiring a
  manual daemon restart.

## main.py 1.3.0

### Added
- MQTT authentication: `MQTTClient` now accepts `MQTT_USER` and `MQTT_PASSWORD` from config
  (`None` = anonymous access preserved for backward compatibility).
- New error code `E--6` on display when MQTT broker rejects credentials (CONNACK rc 4/5).
  Retry is stopped until device restart — bad credentials won't fix themselves.

## main.py 1.2.6

### Added
- MQTT diagnostics: periodically publishes device health as JSON on `DIAG_TOPIC`
  (uptime, free memory, WiFi RSSI, publish buffer size, WiFi/MQTT reconnect counts,
  firmware version). Interval is configurable via `DIAG_INTERVAL` (number of measurement
  cycles, 0 = disabled).

## main.py 1.2.5

### Fixed
- `_close_mqtt()`: explicitly close the socket if `disconnect()` raises — `umqtt.simple`
  does not close the socket when `sock.write()` fails, which could exhaust the lwIP socket
  pool on repeated reconnects and cause persistent MQTT failure (observed in production
  as E--2 with normal sensor readings, cleared by restart).

## main.py 1.2.4

### Fixed
- `_sync_ntp()` now returns `True`/`False` to indicate success. On failure, the daily
  trigger in `run()` falls back to a local DST calculation and updates `_offset_day`,
  preventing repeated 5-second NTP retry attempts for the rest of the day.

## main.py 1.2.3

### Fixed
- Added comment on `client.sock.settimeout()` explaining the intentional use of umqtt.simple internal API (no public alternative exists).
- `CLAUDE.md`: corrected MQTT retain documentation — `{topic}/status` is published with `retain=False` (intentional: subscribers should not receive stale status after reconnect).

## main.py 1.2.2

### Fixed
- `config.py.example`: `MQTT_BROKER` documented as IP-address-only — using a hostname causes `getaddrinfo()` to block indefinitely without feeding the WDT.

## main.py 1.2.1

### Fixed
- `utc_offset` is now recalculated immediately after every NTP sync (previously it could remain wrong for up to ~24 h after boot or WiFi reconnect if the RTC was at epoch before the first sync).
- Daily NTP resync added: once per day at ~01:00 UTC the clock is re-synchronized if WiFi is available, preventing RTC drift (~1.7 s/day) from accumulating in MQTT timestamps.
- `create_sensor()` now catches `ValueError` in addition to `KeyError`, so a misconfigured sensor (e.g. invalid `u_divider`) is skipped rather than crashing the whole application.

## main.py 1.2.0

### Added
- Button on `BUTTON_PIN` cycles display through sensors via hardware IRQ (`micropython.schedule`, 200 ms debounce).
- Per-sensor `led_pin` config key: the active sensor's LED pin is driven HIGH, others LOW.
- Both features are optional — `None` disables them without code changes.

## main.py 1.1.4

### Added
- Multi-sensor support on shared ADS1115 with per-sensor conversion logic.
- Wi-Fi and MQTT reconnect handling with buffered publish retry.
- CET/CEST offset handling for MQTT timestamps.

### Changed
- MQTT connection error path now guards against an uninitialized `client`.
- Documentation was synchronized with current configuration defaults and runtime behavior.

### Fixed
- Improved MQTT bridge compatibility with `paho-mqtt` 1.x and 2.x.

## mqtt_to_postgres.py 1.0.2

### Changed
- Insert now calls the `insert_sensor_value(sensor_name, value, ts)` database function instead of an inline `INSERT … SELECT`.
- Parameter order in the insert queue corrected to match the function signature: `(sensor_name, value, ts)`.

## mqtt_to_postgres.py 1.0.1

### Added
- Dedicated bridge daemon for forwarding MQTT sensor data into PostgreSQL.
- Startup version logging via `VERSION = "1.0.1"`.

### Changed
- README now documents the actual payload format `value timestamp`.

### Fixed
- `on_connect` callback is compatible with both old and new `paho-mqtt` callback signatures.
