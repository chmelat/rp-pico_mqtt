#
# Desktopová kontrola main.py (CPython): python3 selfcheck.py
# Importuje skutečný main.py s náhradami MicroPython modulů a ověřuje logiku,
# která se na zařízení projeví až po dnech: přetečení ticks_ms (audit #8 L1, L8)
# a formát/retain hodnotového payloadu před NTP (L5).
#
import os
import sys
import types
from importlib.machinery import SourceFileLoader

sys.dont_write_bytecode = True  # žádný __pycache__ v repu

HERE = os.path.dirname(os.path.abspath(__file__))
DAY = 86_400_000


def stub(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    sys.modules[name] = mod
    return mod


class MQTTException(Exception):
    pass


calls = {"getaddrinfo": 0, "scan": 0}


def getaddrinfo(*_):
    calls["getaddrinfo"] += 1
    raise OSError("stub: broker nedostupný")


stub("machine", I2C=object, Pin=object, WDT=object, reset=lambda: None)
stub("micropython", schedule=lambda f, a: f(a))
stub("ads1x15", ADS1115=object)
stub("tm1637")
stub("network")
stub("ntptime", host="pool.ntp.org", timeout=None, settime=lambda: None)
stub("usocket", getaddrinfo=getaddrinfo)
stub("umqtt")
sys.modules["umqtt.simple"] = stub("umqtt.simple", MQTTClient=object, MQTTException=MQTTException)

# config.py je v .gitignore (hesla), kontrola jede nad config.py.example
config = types.ModuleType("config")
SourceFileLoader("config", os.path.join(HERE, "config.py.example")).exec_module(config)
sys.modules["config"] = config

sys.path.insert(0, HERE)
import main


class FakeTime:
    """MicroPython ticks_* na 32bit portu: perioda 2**30 ms (12,4 d)"""
    PERIOD, HALF = 1 << 30, 1 << 29

    def __init__(self):
        self.ms = 0
        self.tm = (2026, 10, 20, 12, 0, 0, 1, 293)

    def ticks_ms(self):
        return self.ms % self.PERIOD

    def ticks_add(self, t, d):
        return (t + d) % self.PERIOD

    def ticks_diff(self, a, b):
        return ((a - b + self.HALF) % self.PERIOD) - self.HALF

    def gmtime(self):
        return self.tm

    def sleep_ms(self, ms):
        pass


clock = FakeTime()
main.time = clock


class Wlan:
    def isconnected(self):
        return True


def shared_res():
    """SharedResources bez hardwaru — jen atributy, které testované metody čtou"""
    s = main.SharedResources.__new__(main.SharedResources)
    s.__dict__.update(wdt=None, display=None, wlan=Wlan(), mqtt=None, v_ref=2.048,
                      reconn_wifi=0, reconn_mqtt=0, _mqtt_ever_connected=False,
                      _mqtt_auth_err=False, _mqtt_next_try=0, _mqtt_backoff=1000,
                      _wifi_connecting=False, _last_ntp_day=0, adc=None,
                      _adc_next_try=0, _adc_backoff=1000, _pub_buffer=[])
    return s


def mqtt_attempted(s):
    before = calls["getaddrinfo"]
    s.connect_mqtt()
    return calls["getaddrinfo"] > before


# L1: Wi-Fi reconnect v druhé polovině periody ticks (6,3 d) — MQTT se musí zkusit hned
s = shared_res()
clock.ms = int(6.3 * DAY)
s._wifi_connecting = True
assert s.check_wifi()
assert mqtt_attempted(s), "L1: po Wi-Fi reconnectu na 6,3 d se MQTT nepokusil připojit"

# L1: zombie close po 7 dnech spojení — deadline z posledního selhání je zastaralý
s = shared_res()
s._mqtt_next_try = clock.ticks_add(int(1 * DAY), 1000)
s.mqtt = types.SimpleNamespace(disconnect=lambda: None)
clock.ms = int(8 * DAY)
s._close_mqtt()
assert mqtt_attempted(s), "L1: po _close_mqtt() se zastaralým deadlinem se MQTT nepokusil připojit"

# L1: Wi-Fi naskočí mimo reconnect cyklus (_wifi_connecting=False) se zastaralým deadlinem
s = shared_res()
s._mqtt_next_try = clock.ticks_add(int(1 * DAY), 1000)
clock.ms = int(8 * DAY)
assert s.check_wifi()
assert mqtt_attempted(s), "L1: Wi-Fi mimo reconnect cyklus — MQTT se nepokusil připojit"

# backoff dál funguje: hned po selhání se nezkouší, po uplynutí backoffu ano
assert not mqtt_attempted(s), "backoff: pokus hned po selhání"
clock.ms += 1001
assert mqtt_attempted(s), "backoff: po uplynutí backoffu se nezkusilo"

# NTP_HOST: bez volby zůstává pool.ntp.org, s volbou se použije IP z configu
s._sync_ntp()
assert sys.modules["ntptime"].host == "pool.ntp.org"
config.NTP_HOST = "192.0.2.1"
s._sync_ntp()
assert sys.modules["ntptime"].host == "192.0.2.1", "NTP_HOST z configu nepoužit"
del config.NTP_HOST


# L1: glitch ADC po 6,3 d provozu — re-init se musí zkusit hned
def read_fail(*_):
    raise OSError("stub: I2C glitch")


def scan():
    calls["scan"] += 1
    return []


s = shared_res()
s.adc = types.SimpleNamespace(read=read_fail)
s.i2c = types.SimpleNamespace(scan=scan)
clock.ms = int(6.3 * DAY)
assert s.read_adc_channel(0) == (None, main.ERR_ADC)
s.read_adc_channel(0)
assert calls["scan"] == 1, "L1: po chybě ADC na 6,3 d se re-init nepokusil"

# L8: první stisk tlačítka po 6,3 d se nesmí zahodit, odraz do 200 ms ano
m = main.SensorManager.__new__(main.SensorManager)
m.sensors, m._display_sensor, m._btn_last = [None, None], 0, 0
clock.ms = int(6.3 * DAY)
m._btn_pressed(0)
assert m._display_sensor == 1, "L8: stisk po 6,3 d zahozen"
clock.ms += 50
m._btn_pressed(0)
assert m._display_sensor == 1, "L8: odraz do 200 ms nebyl zahozen"


# L5: payload a retain podle stavu RTC, i v bufferu při selhání publish
class Shared:
    def __init__(self, ok):
        self.ok, self.sent, self._pub_buffer = ok, [], []

    def publish(self, topic, payload, retain=True):
        self.sent.append((topic, payload, retain))
        return self.ok


def value_publish(tm, ok=True):
    ch = main.SensorChannel.__new__(main.SensorChannel)
    ch.__dict__.update(shared=Shared(ok), topic="t", status_topic="t/status",
                       precision=3, last_value=12.3454, last_error=None)
    clock.tm = tm
    ch.publish()
    return ch.shared


synced = (2026, 10, 20, 12, 0, 0, 1, 293)
rtc_default = (2021, 1, 1, 0, 0, 7, 4, 1)
assert value_publish(synced).sent[1] == ("t", "12.345 2026-10-20T12:00:00Z", True)
assert value_publish(rtc_default).sent[1] == ("t", "12.345", False), "L5: před NTP čas nebo retain"
assert value_publish(synced, ok=False)._pub_buffer == [("t", "12.345 2026-10-20T12:00:00Z", True)]
assert value_publish(rtc_default, ok=False)._pub_buffer == [], "L5: hodnota bez času v bufferu"

print("selfcheck OK")
