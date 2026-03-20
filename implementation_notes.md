# Implementační poznámky — Pico 2W / CYW43 / MQTT

Zkušenosti získané při ladění periodických výpadků MQTT spojení.

---

## 1. CYW43 — jednosměrný socket způsobuje periodické dropy

**Symptom:** Každých ~10 minut výpadek spojení (LWT „offline" + „online"),
74sekundová díra v datech. Konzistentní, reproducibilní.

**Příčina:** MQTT klient posílal pouze data (publish, PINGREQ), ale nikdy
nečetl odpovědi brokeru (PINGRESP). Socket byl čistě jednosměrný — jen zápisy.

CYW43 drží TCP spojení jinak u pasivního socketu (jen zápisy) než u aktivního
(obousměrný provoz). Při absenci RX aktivity pravděpodobně dochází k vypršení
interního timeru v CYW43 nebo AP, který jednosměrný TX provoz neresetuje
— přestože data odcházejí každou sekundu.

**Fix:** `mqtt.check_msg()` po každém `mqtt.ping()` — klient přečte PINGRESP,
čímž vznikne obousměrná aktivita na socketu.

**Poučení:** Na Pico W / CYW43 zajistit obousměrnou socketovou aktivitu.
Jednosměrný provoz (jen zápisy) nestačí k udržení stabilního spojení,
i při vysoké frekvenci odesílání.

---

## 2. CYW43 PM disable — musí se aplikovat po každém reconnectu

**Symptom:** PM disable nastavený při bootu se ztratí po prvním reconnectu.

**Příčina:** `wlan.config(pm=0xa11140)` bylo voláno pouze v `connect_wifi()`
(initial connect). Po WiFi dropu a reconnectu přes `start_wifi_reconnect()`
se PM disable znovu nenastavil — CYW43 se vrátil do power-save módu.

**Fix:** Volat `wlan.config(pm=0xa11140)` také v `start_wifi_reconnect()`.

**Poučení:** PM disable na CYW43 nepersistuje přes reconnect cyklus.
Musí se re-aplikovat při každém novém `wlan.connect()`.

---

## 3. Zombie MQTT socket — `wlan.isconnected()` nestačí

**Symptom:** Po WiFi dropu se buffer zásobníku nenaplní — `mqtt.publish()`
vrací True po celý výpadek (data jsou zdánlivě publikována, ale broker je
nikdy neobdrží).

**Příčina:** Při deauthu `wlan.isconnected()` zůstává `True` (CYW43 řeší
reconnect interně, transparentně). MQTT socket je „zombie" — zápisy do TCP
send bufferu CYW43 proběhnou bez chyby. Broker detekuje drop okamžitě (TCP
RST), ale device o tom neví. Po 74 s WiFi reconnectne, CYW43 odešle
nahromaděná data, broker odpoví RST → teprve tehdy přijde OSError.

**Fix:** `client.sock.settimeout(2)` po `client.connect()` + `mqtt.check_msg()`
po `mqtt.ping()`. Čtení ze zombie socketu vyhodí OSError po 2 s → MQTT se
zavře → buffer se začne plnit.

**Poučení:** Na CYW43 nelze detekovat WiFi drop přes `wlan.isconnected()`
ani přes chyby při zápisu na socket. Detekce vyžaduje **čtení** ze socketu
s timeoutem (čekání na odpověď brokeru).

---

## 4. `sock.settimeout()` — vedlejší efekt na stabilitu spojení

`sock.settimeout(2)` mění socket z blocking do timeout módu. Tato změna
pravděpodobně ovlivňuje chování CYW43 driveru na nižší úrovni způsobem,
který přispívá ke stabilitě spojení — přesný mechanismus není znám,
ale empiricky: bez `settimeout` = periodické dropy, s `settimeout` = stabilní.

---

## 5. `wlan.disconnect()` před `wlan.connect()` při reconnectu

Explicitní `wlan.disconnect()` před `wlan.connect()` v reconnect smyčce
**prodlužuje** dobu obnovy. Opakované volání `wlan.connect()` bez předchozího
`disconnect()` je rychlejší — CYW43 driver si poradí s pokračující snahou
o připojení bez resetu stavu.

---

## Shrnutí — checklist pro stabilní MQTT na Pico W

- [ ] `wlan.config(pm=0xa11140)` volat při každém `wlan.connect()` (i reconnect)
- [ ] `client.sock.settimeout(N)` nastavit hned po `client.connect()`
- [ ] Po `mqtt.ping()` vždy číst odpověď: `mqtt.check_msg()`
- [ ] Zásobník pro případ výpadku: buffer + flush po reconnectu
- [ ] Neprovádět `wlan.disconnect()` před reconnect pokusem
