# Audit: mqtt_to_postgres.py

Poslední kolo: #2 (v1.0.6, 2026-04-03). Uzavřené nálezy jsou v `CHANGELOG.md`,
zde zůstávají jen otevřené.

## Otevřené nálezy

### B1 — Race condition na `collections.deque`

**Severity: Low** (praktické riziko pod CPython GIL téměř nulové)

`deque` je thread-safe jen pro atomické operace (`append`, `popleft`). `db_worker`
testuje `bool(insert_queue)` bez zámku. Pod CPython to je bezpečné díky GIL, ale je to
implicitní závislost na implementačním detailu.

### B6 — Tiché zahazování nepřečitatelných MQTT zpráv

**Severity: Low**

Pokud payload nejde rozdělit na `(value, timestamp)`, zpráva se zahodí jen s `log.debug`.
Při změně formátu payloadu vypadá démon zdravě, ale nic nevkládá.

**Fix:** periodický počitadlo zahozených zpráv na úrovni `log.info`.

Sem spadá i S4 z `../AUDIT.md` — diagnostický JSON na `DIAG_TOPIC` je pro tento parser
neplatná zpráva a tiše se zahazuje.

### B7 — Tiché zahazování při plné frontě

**Severity: Low**

`deque(maxlen=10_000)` při zaplnění mlčky zahazuje nejstarší položky. Při dlouhém výpadku
DB je to očekávané chování, ale chybí log nebo metrika signalizující ztrátu dat.

**Fix:** zalogovat dosažení kapacity, aspoň jednou za epizodu.

### B5 — Daemon thread + join timeout = ztráta dat při SIGTERM

**Severity: Medium** — **mitigováno 2026-04-01**

`worker.join(timeout=10)` dá 10 s na flush, pak `sys.exit(0)` daemon vlákno ukončí.
Po mitigaci se počet nezpracovaných položek aspoň zaloguje (journalctl), ztráta dat je
tedy viditelná, ne tichá. Plné řešení by vyžadovalo perzistentní frontu.

## Historie — uzavřené nálezy

| ID | Popis | Stav |
|----|-------|------|
| N1 | `executemany` zahazuje celý batch kvůli jednomu špatnému řádku | OPRAVENO 2026-04-03 (rollback + row-by-row retry) |
| N2 / B4 | `client.connect()` bez timeoutu + signal handlers až po connect | OPRAVENO 2026-04-03 (handlery před `connect()`) |
| N5 / D4 | `db_worker` backoff neresetován po úspěšném insertu | FALSE POSITIVE — transientní chyba vždy vynutí reconnect, ten resetuje backoff |
| B2 | `executemany` + `autocommit` → částečné zápisy a duplikáty | OPRAVENO 2026-04-01 (`autocommit=False` + explicitní `commit()`) |
| B3 | paho-mqtt 2.x callback inkompatibilita | OPRAVENO 2026-04-01 (`_PAHO_V2` detekce + `CallbackAPIVersion.VERSION2`) |
| D3 | Chybí volitelná MQTT autentizace | OPRAVENO (`username`/`password` v `[mqtt]` sekci configu) |

## Ověřená správnost (v1.0.6)

| Oblast | Výsledek |
|--------|----------|
| Parameterized queries (SQL injection safe) | Správně ✓ |
| `on_connect` resubscribuje po reconnectu | Správně ✓ |
| `deque` operace thread-safe pod GIL | Správně ✓ |
| Transientní chyba → batch zpět do fronty (zachovává pořadí) | Správně ✓ |
| `float()` parsing odmítne non-numeric payload | Správně ✓ |
