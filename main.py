#
# Device: RPI Pico 2W
# Podpora více heterogenních senzorů na ADS1115
#
from machine import I2C, Pin, WDT
from ads1x15 import ADS1115
import tm1637
from umqtt.simple import MQTTClient, MQTTException
import network
import ntptime
import time
import math
import gc
import config

VERSION = "1.1.0"

# Napěťový rozsah podle gain
GAIN_VREF = {
    1: 4.096,
    2: 2.048,
    4: 1.024,
    8: 0.512,
    16: 0.256,
}

# Chybové kódy (pro displej, max 4 znaky)
ERR_WIFI = "E--1"
ERR_MQTT = "E--2"
ERR_LO = "E-Lo"
ERR_HI = "E-Hi"
ERR_ADC = "E--3"
ERR_CFG = "E--4"
ERR_FATAL = "E--5"

# Srozumitelné zprávy pro MQTT
ERR_MQTT_MSG = {
    ERR_LO: "sensor_low",
    ERR_HI: "sensor_high",
    ERR_ADC: "adc_error",
    ERR_CFG: "config_error",
}


def _last_sunday(year, month):
    """Vrátí den v měsíci poslední neděle daného měsíce."""
    days_in_month = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    if month == 2 and (year % 4 == 0 and (year % 100 != 0 or year % 400 == 0)):
        last = 29
    else:
        last = days_in_month[month]
    t = time.gmtime(time.mktime((year, month, last, 0, 0, 0, 0, 0)))
    # weekday: 0=Po, 6=Ne
    return last - (t[6] + 1) % 7


def cet_offset(t):
    """Vrátí UTC offset v sekundách: 7200 (CEST, léto) nebo 3600 (CET, zima)."""
    year, month, day, hour = t[0], t[1], t[2], t[3]
    if month < 3 or month > 10:
        return 3600
    if 3 < month < 10:
        return 7200
    ls = _last_sunday(year, month)
    if month == 3:
        return 7200 if (day > ls or (day == ls and hour >= 1)) else 3600
    else:  # říjen
        return 3600 if (day > ls or (day == ls and hour >= 1)) else 7200


class SharedResources:
    """Sdílené zdroje pro všechny senzory"""

    def __init__(self):
        self.wdt = None
        self.v_ref = GAIN_VREF.get(config.ADC_GAIN)
        t = time.gmtime()
        self.utc_offset = cet_offset(t)
        self._offset_day = t[2]

        # Display jako první - pro zobrazení chyb při inicializaci
        try:
            self.display = tm1637.TM1637Decimal(clk=Pin(config.TM_CLK),
                                                dio=Pin(config.TM_DIO))
            self.display.brightness(7)
            self.display.show(VERSION)
            time.sleep_ms(2000)
        except Exception as e:
            print("Display init error:", e)
            self.display = None

        # Wi-Fi
        self.wlan = network.WLAN(network.STA_IF)
        if not self.connect_wifi():
            self.show_error(ERR_WIFI)
            self.safe_sleep_ms(2000)

        # I2C a ADC
        self.i2c = I2C(config.I2C_ID,
                       sda=Pin(config.I2C_SDA),
                       scl=Pin(config.I2C_SCL),
                       freq=config.I2C_FREQ)
        self.adc = None
        self._adc_next_try = 0
        self._adc_backoff = 1000
        self._init_adc()

        # MQTT
        self.mqtt = None
        self._mqtt_next_try = 0
        self._mqtt_backoff = 1000
        self._mqtt_last_ping = 0
        self._wifi_connecting = False
        self._wifi_deadline = 0
        self._pub_buffer = []  # zásobník: [(topic, payload, retain), ...]

    def safe_sleep_ms(self, ms):
        """Sleep s krmením watchdogu"""
        while ms > 0:
            if self.wdt:
                self.wdt.feed()
            chunk = min(ms, config.WDT_TIMEOUT_MS // 2)
            time.sleep_ms(chunk)
            ms -= chunk

    def connect_wifi(self):
        """Připojení k Wi-Fi"""
        self.wlan.active(True)
        self.wlan.config(pm=0xa11140)  # zakázat power management CYW43 (prevence deauth od AP)
        if not self.wlan.isconnected():
            self.wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)

        for _ in range(20):  # 20 × 0.5s = max 10s
            if self.wlan.isconnected():
                break
            self.safe_sleep_ms(500)

        if self.wlan.isconnected():
            print("WiFi OK:", self.wlan.ifconfig()[0])
            self._sync_ntp()
            return True
        else:
            print("WiFi CHYBA")
            return False

    def _sync_ntp(self):
        """Synchronizace RTC přes NTP"""
        try:
            ntptime.timeout = 5
            ntptime.settime()
            print("NTP OK:", time.gmtime()[:6])
        except Exception as e:
            print("NTP CHYBA:", e)

    def start_wifi_reconnect(self):
        """Zahájení non-blocking Wi-Fi reconnectu"""
        if self._wifi_connecting:
            return
        self.wlan.active(True)
        self.wlan.config(pm=0xa11140)
        if not self.wlan.isconnected():
            self.wlan.disconnect()
            self.wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
            self._wifi_connecting = True
            self._wifi_deadline = time.ticks_add(time.ticks_ms(), 30_000)

    def check_wifi(self):
        """Kontrola Wi-Fi připojení"""
        if self.wlan.isconnected():
            if self._wifi_connecting:
                self._mqtt_backoff = 1000
                self._mqtt_next_try = 0
                self._sync_ntp()
            self._wifi_connecting = False
            return True
        if self._wifi_connecting:
            if time.ticks_diff(time.ticks_ms(), self._wifi_deadline) > 0:
                print("WiFi CHYBA")
                self._wifi_connecting = False
        return False

    def _init_adc(self):
        """Inicializace ADC s exponenciálním backoffem"""
        now = time.ticks_ms()
        if time.ticks_diff(now, self._adc_next_try) < 0:
            return

        try:
            # Ověření přítomnosti ADC na I2C sběrnici
            devices = self.i2c.scan()
            if config.ADC_ADDRESS not in devices:
                if devices:
                    found = ["0x{:02X}".format(d) for d in devices]
                else:
                    found = "nic"
                print("ADC nenalezen na 0x{:02X}, nalezeno: {}".format(
                    config.ADC_ADDRESS, found))
                raise OSError("ADC not found")

            self.adc = ADS1115(self.i2c, address=config.ADC_ADDRESS, gain=config.ADC_GAIN)
            self._adc_backoff = 1000
        except Exception as e:
            print("ADC init error:", e)
            self.show_error(ERR_ADC)
            self.adc = None
            self._adc_next_try = time.ticks_add(now, self._adc_backoff)
            self._adc_backoff = min(self._adc_backoff * 2, 60_000)

    def read_adc_channel(self, channel):
        """Čtení raw hodnoty z ADC kanálu s průměrováním, vrací (raw, error)"""
        if self.v_ref is None:
            return None, ERR_CFG
        if self.adc is None:
            self._init_adc()
            if self.adc is None:
                return None, ERR_ADC
        samples = config.ADC_SAMPLES
        vals = []
        for i in range(samples):
            try:
                raw = self.adc.read(config.ADC_RATE, channel)
                vals.append(raw)
            except OSError as e:
                print("ADC error:", e)
                self.adc = None
                break
            if i % 10 == 9 and self.wdt:
                self.wdt.feed()
        if not vals:
            return None, ERR_ADC
        return sum(vals) // len(vals), None

    def _close_mqtt(self):
        """Uzavření MQTT spojení a uvolnění socketu"""
        if self.mqtt is not None:
            try:
                self.mqtt.disconnect()
            except Exception:
                pass
            self.mqtt = None

    def connect_mqtt(self):
        """Připojení k MQTT brokeru s exponenciálním backoffem"""
        if self.mqtt is not None:
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, self._mqtt_next_try) < 0:
            return False

        if self.wdt:
            self.wdt.feed()

        # TCP pre-test s timeoutom – predíde blokovaniu WDT pri nedostupnom brokeri
        import usocket
        try:
            addr = usocket.getaddrinfo(config.MQTT_BROKER, config.MQTT_PORT)[0][-1]
            s = usocket.socket()
            try:
                s.settimeout(4)
                s.connect(addr)
            finally:
                try:
                    s.close()
                except OSError:
                    pass
        except OSError as e:
            print("MQTT broker nedostupný:", e)
            self._mqtt_next_try = time.ticks_add(now, self._mqtt_backoff)
            self._mqtt_backoff = min(self._mqtt_backoff * 2, 60_000)
            return False

        if self.wdt:
            self.wdt.feed()

        try:
            client = MQTTClient(config.MQTT_CLIENT_ID,
                                config.MQTT_BROKER,
                                config.MQTT_PORT,
                                keepalive=config.MQTT_KEEPALIVE)
            client.set_last_will(config.MQTT_STATUS_TOPIC,
                                 "offline", retain=True)
            client.connect()
            client.publish(config.MQTT_STATUS_TOPIC, "online", retain=True)
            self.mqtt = client
            self._mqtt_backoff = 1000
            self._mqtt_last_ping = time.ticks_ms()
            return True
        except BaseException as e:
            if isinstance(e, (MemoryError, KeyboardInterrupt, SystemExit)):
                raise
            print("MQTT error:", e)
            try:
                client.disconnect()
            except BaseException:
                pass
            self._mqtt_next_try = time.ticks_add(now, self._mqtt_backoff)
            self._mqtt_backoff = min(self._mqtt_backoff * 2, 60_000)
            return False

    def mqtt_ping(self):
        """Periodický MQTT ping pro udržení spojení"""
        # Pozn.: umqtt.simple neověřuje PINGRESP — při half-open TCP
        # (NAT expiry aj.) zůstane spojení "zombie". V LAN akceptovatelné;
        # OSError z publish/ping detekuje většinu výpadků.
        if self.mqtt is None:
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, self._mqtt_last_ping) < config.MQTT_KEEPALIVE * 500:
            return
        try:
            self.mqtt.ping()
            self._mqtt_last_ping = now
        except (OSError, MQTTException):
            self._close_mqtt()

    def publish(self, topic, payload, retain=True):
        """Odeslání payload na MQTT topic"""
        if not self.connect_mqtt():
            return False
        try:
            self.mqtt.publish(topic, str(payload), retain=retain)
            return True
        except (OSError, MQTTException):
            self._close_mqtt()
            return False

    def flush_buffer(self):
        """Odeslání zpráv ze zásobníku, nejstarší první"""
        if not self._pub_buffer or not self.connect_mqtt():
            return
        sent = 0
        while self._pub_buffer:
            if self.wdt and sent % 10 == 0:
                self.wdt.feed()
            topic, payload, retain = self._pub_buffer[0]
            try:
                self.mqtt.publish(topic, payload, retain=retain)
                self._pub_buffer.pop(0)
                sent += 1
            except (OSError, MQTTException):
                self._close_mqtt()
                break
        if sent:
            print("Zásobník: odesláno", sent, "zpráv, zbývá", len(self._pub_buffer))

    def show_value(self, value):
        """Zobrazení hodnoty na displeji"""
        if not self.display:
            return
        if value < -99 or value >= 1000:
            self.display.show("----")
        elif value < 0:
            # Záporné: max 2 desetinná místa (např. -9.99)
            if value <= -10:
                self.display.show("{:.1f}".format(value))
            else:
                self.display.show("{:.2f}".format(value))
        elif value >= 100:
            self.display.show("{:.1f}".format(value))
        elif value >= 10:
            self.display.show("{:.2f}".format(value))
        else:
            self.display.show("{:.3f}".format(value))

    def show_error(self, code):
        """Zobrazení chyby na displeji"""
        if not self.display:
            return
        self.display.show(code)


class SensorChannel:
    """Základní třída pro senzorový kanál"""

    def __init__(self, shared, cfg):
        self.shared = shared
        self.channel = cfg["channel"]
        self.topic = cfg["topic"]
        self.status_topic = cfg.get("status_topic", cfg["topic"] + "/status")
        self.name = cfg.get("name", "CH{}".format(self.channel))
        self.precision = cfg.get("precision", 3)
        self.last_value = None
        self.last_error = None

    def convert_raw(self, raw):
        """Převod raw ADC hodnoty na výstupní hodnotu — přepíše potomek"""
        raise NotImplementedError

    def read(self):
        """Čtení hodnoty, vrací (hodnota, chyba)"""
        raw, error = self.shared.read_adc_channel(self.channel)
        if error:
            self.last_value, self.last_error = None, error
            return None, error
        value, error = self.convert_raw(raw)
        self.last_value, self.last_error = value, error
        return value, error

    def publish(self):
        """Publikování hodnoty a stavu na oddělené topicy"""
        ok = True

        # Stav: "OK" nebo srozumitelná chybová zpráva pro MQTT
        if self.last_error:
            status = ERR_MQTT_MSG.get(self.last_error, self.last_error)
        else:
            status = "OK"
        if not self.shared.publish(self.status_topic, status, retain=False):
            ok = False

        # Hodnotu publikuj jen když je platná
        if self.last_value is not None:
            t = time.gmtime(time.mktime(time.gmtime()) + self.shared.utc_offset)
            ts = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}".format(
                t[0], t[1], t[2], t[3], t[4], t[5])
            formatted = "{:.{}f} {}".format(self.last_value, self.precision, ts)
            if not self.shared.publish(self.topic, formatted, retain=True):
                buf = self.shared._pub_buffer
                if len(buf) >= config.PUBLISH_BUFFER_MAX:
                    buf.pop(0)
                buf.append((self.topic, formatted, True))
                ok = False

        return ok


class CurrentLoopSensor(SensorChannel):
    """Senzor s 4-20mA proudovou smyčkou, lineární převod na tlak"""

    def __init__(self, shared, cfg):
        super().__init__(shared, cfg)
        self.r_bocnik = cfg["r_bocnik"]
        self.i_min = cfg.get("i_min", 0.004)
        self.i_max = cfg.get("i_max", 0.020)
        self.p_min = cfg["p_min"]
        self.p_max = cfg["p_max"]
        self.v_min = self.i_min * self.r_bocnik
        self.v_max = self.i_max * self.r_bocnik

    def convert_raw(self, raw):
        """Převod raw ADC hodnoty na tlak"""
        voltage = raw * self.shared.v_ref / 32767
        if voltage < 0:
            return None, ERR_LO

        if voltage < self.v_min * 0.8:
            return None, ERR_LO
        if voltage > self.v_max * 1.1:
            return None, ERR_HI

        v_range = self.v_max - self.v_min
        if v_range == 0:
            return None, ERR_CFG

        pressure = self.p_min + (voltage - self.v_min) / v_range * (self.p_max - self.p_min)

        if pressure < self.p_min:
            return None, ERR_LO
        if pressure > self.p_max:
            return None, ERR_HI

        return round(pressure, self.precision), None


class PiraniSensor(SensorChannel):
    """Pirani vakuometr, převod napětí na tlak: p = exp(a + b*u + c*sqrt(u))"""

    def __init__(self, shared, cfg):
        super().__init__(shared, cfg)
        self.a = cfg["a"]
        self.b = cfg["b"]
        self.c = cfg["c"]
        self.u_divider = cfg.get("u_divider", 1.0)
        self.u_min = cfg.get("u_min", 0.0)
        self.u_max = cfg.get("u_max", shared.v_ref or 2.048)
        self.p_min = cfg.get("p_min", 1e-4)
        self.p_max = cfg.get("p_max", 1000.0)

    def convert_raw(self, raw):
        """Převod raw ADC hodnoty na tlak přes regresní model"""
        voltage = raw * self.shared.v_ref / 32767
        if voltage < 0:
            return None, ERR_LO

        if voltage < self.u_min:
            return None, ERR_LO
        if voltage > self.u_max:
            return None, ERR_HI

        try:
            u_actual = voltage * self.u_divider
            pressure = math.exp(self.a + self.b * u_actual + self.c * math.sqrt(u_actual))
        except OverflowError:
            return None, ERR_HI

        if pressure < self.p_min:
            return None, ERR_LO
        if pressure > self.p_max:
            return None, ERR_HI

        return round(pressure, self.precision), None


# Factory pro vytváření senzorů
SENSOR_TYPES = {
    "current_loop": CurrentLoopSensor,
    "pirani": PiraniSensor,
}


def create_sensor(shared, cfg):
    """Vytvoření senzoru podle typu v konfiguraci, vrací None při chybě"""
    sensor_type = cfg.get("type", "current_loop")  # vyber sensor podle typu, pri neuspechu "current_loop"
    cls = SENSOR_TYPES.get(sensor_type) # vytahni do cls tridu podle typu,
    if cls is None:
        print("Neznámý typ senzoru:", sensor_type)
        return None
    try:
        return cls(shared, cfg)
    except KeyError as e:
        print("Chybí klíč v konfiguraci senzoru:", e)
        return None


class SensorManager:
    """Správce senzorů - hlavní smyčka"""

    def __init__(self):
        self.shared = SharedResources()
        self.sensors = []
        for cfg in config.SENSORS:
            sensor = create_sensor(self.shared, cfg)
            if sensor is not None:
                self.sensors.append(sensor)

        # Výběr senzoru pro displej (omezení na platný rozsah)
        if self.sensors:
            max_index = len(self.sensors) - 1
            self._display_sensor = max(0, min(config.DISPLAY_SENSOR, max_index))
        else:
            self._display_sensor = 0

        self._conn_error = None  # ERR_WIFI nebo ERR_MQTT

    def _read_all(self):
        """Čtení ze všech senzorů"""
        for s in self.sensors:
            s.read()

    def _publish_all(self):
        """Publikování hodnot ze všech senzorů, vrací True pokud vše OK"""
        all_ok = True
        for s in self.sensors:
            if not s.publish():
                all_ok = False
        return all_ok

    def _update_display(self):
        """Zobrazení vybraného senzoru, střídá s chybou připojení"""
        if not self.shared.display or not self.sensors:
            return

        s = self.sensors[self._display_sensor]

        # Pokud je chyba připojení, střídáme hodnotu (2s) a chybu (0.5s)
        has_conn_error = self._conn_error is not None
        has_valid_value = s.last_value is not None
        has_no_sensor_error = not s.last_error

        if has_conn_error and has_valid_value and has_no_sensor_error:
            cycle = time.ticks_ms() % 2500
            if cycle >= 2000:
                self.shared.show_error(self._conn_error)
                return

        if s.last_error:
            self.shared.show_error(s.last_error)
        elif s.last_value is not None:
            self.shared.show_value(s.last_value)

    def run(self):
        
        self.shared.wdt = WDT(timeout=config.WDT_TIMEOUT_MS)

        if not self.shared.connect_mqtt():
            self.shared.show_error(ERR_MQTT)
            self.shared.safe_sleep_ms(2000)

        while True:
            self.shared.wdt.feed()
            loop_start = time.ticks_ms()

            t = time.gmtime()
            if t[3] >= 1 and t[2] != self.shared._offset_day:
                self.shared.utc_offset = cet_offset(t)
                self.shared._offset_day = t[2]

            wifi_ok = self.shared.check_wifi()
            if not wifi_ok:
                self.shared._close_mqtt()
                self.shared.start_wifi_reconnect()

            self._read_all()

            if wifi_ok:
                self.shared.flush_buffer()
                publish_ok = self._publish_all()
                if not publish_ok:
                    self._conn_error = ERR_MQTT
                else:
                    self._conn_error = None
                self.shared.mqtt_ping()
            else:
                self._conn_error = ERR_WIFI

            self._update_display()

            elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
            remaining = config.INTERVAL_S * 1000 - elapsed
            if remaining > 0:
                gc.collect()
                self.shared.safe_sleep_ms(remaining)


if __name__ == "__main__":
    try:
        SensorManager().run()
    except Exception as e:
        print("FATAL:", e)
        try:
            d = tm1637.TM1637Decimal(clk=Pin(config.TM_CLK),
                                      dio=Pin(config.TM_DIO))
            d.show(ERR_FATAL)
        except Exception:
            pass
        # WDT resetuje zařízení
