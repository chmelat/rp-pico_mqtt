# Changelog

All notable changes to this project will be documented in this file.

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

## mqtt_to_postgres.py 1.0.1

### Added
- Dedicated bridge daemon for forwarding MQTT sensor data into PostgreSQL.
- Startup version logging via `VERSION = "1.0.1"`.

### Changed
- README now documents the actual payload format `value timestamp`.

### Fixed
- `on_connect` callback is compatible with both old and new `paho-mqtt` callback signatures.
