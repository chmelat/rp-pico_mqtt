# Changelog

All notable changes to this project will be documented in this file.

## Documentation

### Added
- README hardware section "Ochrana vstupů ADC (přepětí 15 V)" — documents that the
  4-20 mA loop's ~15 V supply, present on the sensor connector, can reach the ADS1115
  input via a shorted connector and destroy it. Zener clamps (2.4V, then 3.3V) leaked
  in the 0-2 V signal band and distorted readings (visible at ~100 kPa on a 0-120 kPa
  range); all were removed (6/2026). Recommends a Schottky-to-VDD clamp behind the
  series resistor instead — references VDD rather than its own breakdown, so it does
  not leak below VDD while still holding the pin to ~VDD+0.3 V on a 15 V fault.

## main.py 1.5.0

### Added
- New error code `E-nc` (sensor disconnected) for `CurrentLoopSensor` — fires when
  measured current is below `i_disconnect` (default 0.001 A = 1 mA). In a live
  4-20 mA loop, current below ~1 mA is physically impossible (per NAMUR NE43 even
  a faulting transmitter holds ≥3.5 mA), so this reliably indicates a broken wire,
  unpowered sensor, or disconnected loop — distinct from `E-Lo` which now means
  "sensor works but reads below the configured range."
- New per-sensor config key `i_disconnect` (Amps). Set to `None` to disable the
  check. Applies only to `current_loop`; `pirani` is voltage-based and behaves
  differently on disconnect.
- MQTT status string `sensor_disconnected` published on `{topic}/status` when
  `E-nc` is active.

## main.py 1.4.0

### Changed
- MQTT publish timestamps are now UTC with `Z` suffix (e.g. `2026-04-18T12:00:00Z`)
  instead of naked local time (CET/CEST without TZ marker). Postgres interprets the
  explicit UTC correctly regardless of session TZ, eliminating a lurking silent-shift
  bug if the DB session TZ ever changed. Grafana/psql display via session TZ
  continues to show local time — no visible change in dashboards or queries.
- Matches the fallback format already used by `mqtt_to_postgres.py` when MQTT payload
  contains no timestamp — a single consistent shape in the `create_tms` column.

### Removed
- DST infrastructure: `_last_sunday()` function, `cet_offset()` function,
  `SharedResources.utc_offset` attribute, and the DST portion of the daily resync
  in `run()`. No longer needed — `time.gmtime()` for UTC publish does not depend on
  local zone.

### Kept
- Daily NTP RTC resync triggered after 01:00 UTC on a new day — unchanged behavior,
  only the DST offset recomputation has been removed. `SharedResources._offset_day`
  renamed to `_last_ntp_day` to reflect its remaining role (tracking last resync).

## main.py 1.3.1

### Fixed
- WDT starvation in `SharedResources.publish()` — now feeds the watchdog before each
  publish attempt. Previously, a chain of 2 publishes per sensor × N sensors × 2s socket
  timeout could exceed `WDT_TIMEOUT_MS` on slow or unresponsive networks (audit #6, A2).
- WDT starvation in `SharedResources.flush_buffer()` — feed now runs every iteration
  instead of every 10th. Long backlog flushes over a slow link no longer risk reset
  (audit #6, A1; previously tracked as N3/R1 in audits #4 and #5).

## mqtt_to_postgres.py 1.2.1

### Added
- `psycopg2.extras.execute_batch` replaces `cursor.executemany` for batch inserts —
  significantly fewer round-trips to the DB per batch.
- Event-driven DB worker: `threading.Condition` replaces the 0.5s polling loop.
  `on_message` notifies on queue append; worker sleeps until data arrives. Lower CPU
  at idle and lower insert latency.

### Fixed
- Preserved auth-failure handling (CONNACK rc 4/5) from v1.1.x after the Condition
  rewrite — daemon disconnects instead of looping on bad credentials.
- Preserved `on_disconnect` logging — unexpected broker disconnects are logged again.
- Preserved shutdown queue-drain warning — logs count of unflushed items when stopping
  with non-empty queue.
- Restored saved/discarded summary in row-by-row retry path — single INFO line at the
  end instead of per-row ERROR spam; inner except narrowed to
  `(RaiseException, IntegrityError)` so unknown errors propagate to reconnect.
- Removed `try/except` around `client.loop_forever()` — paho handles network retries
  internally; the wrapper masked legitimate errors and duplicated the SIGTERM handler.

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
