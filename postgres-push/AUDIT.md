# Audit: mqtt_to_postgres.py

---

## Audit #2 — hloubkový audit (v1.0.6)

**Date:** 2026-04-03

### Souhrn závažnosti

| ID | Závažnost | Popis | Stav |
|----|-----------|-------|------|
| N1 | ~~STŘEDNÍ~~ | `executemany` zahazuje celý batch kvůli jednomu špatnému řádku | **OPRAVENO** |
| N2 | ~~STŘEDNÍ~~ | `client.connect()` bez timeoutu + signal handlers až po connect | **OPRAVENO** |
| N5 | ~~NÍZKÁ~~ | `db_worker` backoff neresetován po úspěšném insertu | **False positive** |

---

### N1 — `executemany` zahazuje celý batch kvůli jednomu špatnému řádku — OPRAVENO

**Severity: Medium** — **Fixed 2026-04-03**

`executemany` vkládá až 500 řádků v jedné transakci. Pokud `insert_sensor_value` vyhodí `RaiseException` pro jeden řádek (neznámý senzor, neplatný timestamp), celý batch se zahazoval.

**Resolution:** Po `RaiseException`/`IntegrityError` se batch rollbackne a zkusí se row-by-row retry. Každý řádek vlastní transakce — dobrý commit, špatný rollback + discard. Žádný řádek se nevrací do fronty → nedochází k duplikátům (neporušuje atomicitu opravu B2).

---

### N2 — `client.connect()` bez timeoutu + signal handlers až po connect — OPRAVENO

**Severity: Medium** — **Fixed 2026-04-03**

`client.connect(broker, port)` blokoval na TCP connect bez explicitního timeoutu. Signal handlers se registrovaly až po connect. Pokud byl broker nedostupný, SIGTERM/SIGINT nemohl proces gracefully ukončit.

B4 z auditu #1.

**Resolution:** Signal handlers přesunuty před `client.connect()`.

---

### N5 — `db_worker` backoff neresetován po úspěšném insertu — FALSE POSITIVE

Transientní chyba vždy nastaví `conn = None`, což vynutí reconnect. Úspěšný reconnect resetuje `backoff = 1` (line 54). Backoff nikdy nezůstane navýšený po obnovení spojení.

D4 z auditu #1 — uzavřeno.

---

### Ověřená správnost (v1.0.6)

| Oblast | Výsledek |
|--------|----------|
| Parameterized queries (SQL injection safe) | Správně ✓ |
| `on_connect` resubscribes po reconnectu | Správně ✓ |
| `deque` operace thread-safe pod GIL | Správně ✓ |
| Batch split TS/no-TS pro dva SQL dotazy | Správně ✓ |
| Transientní chyba → batch zpět do fronty (zachovává pořadí) | Správně ✓ |

### Stav předchozích otevřených nálezů

| ID | Popis | Stav |
|----|-------|------|
| B1 | `deque` thread safety relies on GIL | Otevřeno (Low) |
| B4 | Signal handler po connect | **Opraveno** → N2 |
| B6 | Silent drop špatných MQTT zpráv | Otevřeno (Low) |
| B7 | Silent discard při plné queue | Otevřeno (Low) |
| D3 | Chybí MQTT autentizace | Otevřeno (INFO) |
| D4 | Backoff neresetován po insertu | **False positive** → N5 |

---

## Audit #1 — v1.0.3

**Date:** 2026-04-01

---

## B1 — Race condition na `collections.deque`

**Severity: Low (practical risk near zero under CPython GIL)**

`deque` is thread-safe only for atomic operations (`append`, `popleft`). The `db_worker` checks `bool(insert_queue)` without a lock (lines 44, 59). Under CPython this is safe due to the GIL, but it is an implicit dependency on an implementation detail.

## ~~B2 — `executemany` + `autocommit` = partial batch writes~~ FIXED

**Severity: Medium** — **Fixed 2026-04-01**

`conn.autocommit = True` (line 49) combined with `cur.executemany()` (line 69) means each row in a batch is its own transaction. If the connection drops mid-batch (e.g. after 200 of 500 rows), the first 200 are committed, but the entire batch of 500 is returned to the queue (line 77) and will be re-inserted — **causing duplicates**.

**Resolution:** Changed to `autocommit = False` with explicit `conn.commit()` after successful `executemany`. Batch is now atomic — either all rows commit or none do.

## ~~B3 — paho-mqtt v2.x incompatibility~~ FIXED

**Severity: Medium** — **Fixed 2026-04-01**

- `mqtt.Client()` (line 140) without `CallbackAPIVersion` triggers a `DeprecationWarning` or error in paho-mqtt >= 2.0.
- `on_disconnect(client, userdata, rc)` (line 131) lacks the `*args` catch present in `on_connect` (line 124), so it will break under v2 callback conventions.

**Resolution:** Added runtime detection (`_PAHO_V2 = hasattr(mqtt, "CallbackAPIVersion")`) and conditional constructor using `CallbackAPIVersion.VERSION2` for paho v2, with callback signatures compatible with both v1 and v2.

## B4 — Signal handler registered after `connect()`

**Severity: Low-Medium**

`signal.signal(SIGTERM, shutdown)` is registered on lines 154-155, **after** `client.connect()` on line 144. If the broker is unreachable and `connect()` blocks, SIGTERM will not be caught and the process cannot shut down gracefully.

**Fix:** Register signal handlers before `client.connect()`.

## ~~B5 — Daemon thread + join timeout = data loss on SIGTERM~~ MITIGATED

**Severity: Medium** — **Mitigated 2026-04-01**

The DB worker is started as `daemon=True` (line 136). In `shutdown`, `worker.join(timeout=10)` gives 10 seconds to flush, then `sys.exit(0)` kills the daemon thread. If the queue has thousands of items and the DB is slow, pending data is lost silently.

**Resolution:** Added warning log after `worker.join` that reports the number of unflushed items if any remain. Data loss is still possible but no longer silent — it will appear in journalctl/syslog.

## B6 — Silent drop of unparseable MQTT messages

**Severity: Low**

Lines 114-119: if payload cannot be split into `(value, timestamp)`, the message is silently discarded with no logging. If the payload format changes, the daemon appears healthy but inserts nothing.

**Fix:** Add a `log.debug` or periodic counter for dropped messages.

## B7 — Silent discard when queue is full

**Severity: Low**

`deque(maxlen=10_000)` (line 107) silently drops the oldest items when full. During prolonged DB outages this is expected behavior, but there is no log or metric indicating data loss.

**Fix:** Log when the queue reaches capacity, at least once per episode.

---

## Summary

| #    | Issue                                              | Severity    |
| ---- | -------------------------------------------------- | ----------- |
| B2   | ~~`executemany` + `autocommit` causes duplicates~~  | **Fixed**   |
| B3   | ~~paho-mqtt v2.x callback incompatibility~~         | **Fixed**   |
| B5   | daemon thread + timeout join = silent data loss    | **Mitigated** |
| B4   | Signal handler after connect = missed SIGTERM      | Low-Medium  |
| B1   | deque thread safety relies on GIL                  | Low         |
| B6   | Silent drop of bad MQTT messages                   | Low         |
| B7   | Silent discard on full queue                        | Low         |
