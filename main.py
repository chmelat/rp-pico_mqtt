#
# Device: RPI Pico 2W
# Podpora více heterogenních senzorů na ADS1115
#
from machine import I2C, Pin, WDT, reset
import micropython
from ads1x15 import ADS1115
import tm1637
from umqtt.simple import MQTTClient, MQTTException
import network
import ntptime
import time
import math
import gc
import json
import usocket
import config

VERSION = "1.5.2"

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
ERR_AUTH = "E--6"
ERR_NC = "E-nc"

# Srozumitelné zprávy pro MQTT
ERR_MQTT_MSG = {
    ERR_LO: "sensor_low",
    ERR_HI: "sensor_high",
    ERR_ADC: "adc_error",
    ERR_CFG: "config_error",
    ERR_NC: "sensor_disconnected",
}


class SharedResources:
    """Sdílené zdroje pro všechny senzory"""

    def __init__(self):
        self.wdt = None
        self.reconn_wifi = 0
        self.reconn_mqtt = 0
        self._mqtt_ever_connected = False
        self.v_ref = GAIN_VREF.get(config.ADC_GAIN)
        self._last_ntp_day = time.gmtime()[2]

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
        self._mqtt_auth_err = False
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
        """Synchronizace RTC přes NTP. Vrátí True při úspěchu."""
        try:
            ntptime.timeout = 5
            ntptime.settime()
            t = time.gmtime()
            self._last_ntp_day = t[2]
            print("NTP OK:", t[:6])
            return True
        except Exception as e:
            print("NTP CHYBA:", e)
            return False

    def start_wifi_reconnect(self):
        """Zahájení non-blocking Wi-Fi reconnectu"""
        if self._wifi_connecting:
            return
        self.wlan.active(True)
        self.wlan.config(pm=0xa11140)
        if not self.wlan.isconnected():
            self.wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
            self._wifi_connecting = True
            self._wifi_deadline = time.ticks_add(time.ticks_ms(), 90_000)

    def check_wifi(self):
        """Kontrola Wi-Fi připojení"""
        if self.wlan.isconnected():
            if self._wifi_connecting:
                self._wifi_connecting = False
                self.reconn_wifi += 1
                self._mqtt_backoff = 1000
                self._mqtt_next_try = 0
                self._sync_ntp()
            return True
        if self._wifi_connecting:
            if time.ticks_diff(time.ticks_ms(), self._wifi_deadline) > 0:
                print("WiFi deadline — nový pokus")
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
        return sum(vals) / len(vals), None

    def _close_mqtt(self):
        """Uzavření MQTT spojení a uvolnění socketu"""
        if self.mqtt is not None:
            try:
                self.mqtt.disconnect()
            except Exception:
                try:
                    self.mqtt.sock.close()  # záloha: disconnect() socket nezavře pokud selže write()
                except Exception:
                    pass
            self.mqtt = None

    def connect_mqtt(self):
        """Připojení k MQTT brokeru s exponenciálním backoffem"""
        if self.mqtt is not None:
            return True
        if self._mqtt_auth_err:
            return False
        if not self.wlan.isconnected():
            return False

        now = time.ticks_ms()
        if time.ticks_diff(now, self._mqtt_next_try) < 0:
            return False

        if self.wdt:
            self.wdt.feed()

        # TCP pre-test s timeoutom – predíde blokovaniu WDT pri nedostupnom brokeri
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

        client = None
        try:
            mqtt_user = getattr(config, 'MQTT_USER', None)
            mqtt_pass = getattr(config, 'MQTT_PASSWORD', None) or ""
            client = MQTTClient(config.MQTT_CLIENT_ID,
                                config.MQTT_BROKER,
                                config.MQTT_PORT,
                                user=mqtt_user,
                                password=mqtt_pass if mqtt_user else None,
                                keepalive=config.MQTT_KEEPALIVE)
            client.set_last_will(config.MQTT_STATUS_TOPIC,
                                 "offline", retain=True)
            client.connect(timeout=4)  # umqtt nastaví timeout před sock.connect() i před čtením CONNACK — bez toho blokuje mrtvý broker do WDT resetu
            client.sock.settimeout(2)  # provozní timeout pro publish/ping; přístup k internímu atributu — umqtt jinou cestu nenabízí
            client.publish(config.MQTT_STATUS_TOPIC, "online", retain=True)
            self.mqtt = client
            if self._mqtt_ever_connected:
                self.reconn_mqtt += 1
            self._mqtt_ever_connected = True
            self._mqtt_backoff = 1000
            self._mqtt_last_ping = time.ticks_ms()
            return True
        except BaseException as e:
            if isinstance(e, (MemoryError, KeyboardInterrupt, SystemExit)):
                raise
            if client is not None:
                try:
                    client.disconnect()
                except BaseException:
                    pass
            # CONNACK rc 4 = bad credentials, 5 = not authorized
            if isinstance(e, MQTTException) and e.args and e.args[0] in (4, 5):
                print("MQTT auth failed:", e)
                self._mqtt_auth_err = True
                return False
            print("MQTT error:", e)
            self._mqtt_next_try = time.ticks_add(now, self._mqtt_backoff)
            self._mqtt_backoff = min(self._mqtt_backoff * 2, 60_000)
            return False

    def mqtt_ping(self):
        """Periodický MQTT ping s ověřením PINGRESP — detekuje zombie spojení"""
        if self.mqtt is None:
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, self._mqtt_last_ping) < config.MQTT_KEEPALIVE * 500:
            return
        if self.wdt:
            self.wdt.feed()
        try:
            self.mqtt.ping()
            self.mqtt.check_msg()    # čte PINGRESP; socket timeout 2s → OSError při zombie
            self._mqtt_last_ping = now
        except (OSError, MQTTException):
            self._close_mqtt()

    def publish(self, topic, payload, retain=True):
        """Odeslání payload na MQTT topic"""
        if self.wdt:
            self.wdt.feed()
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
            if self.wdt:
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
        if -99 <= value < 1000:
            for p in (3, 2, 1, 0):
                s = "{:.{}f}".format(value, p)
                if len(s.replace(".", "")) <= 4:  # tečka má vlastní segment
                    self.display.show(s)
                    return
        self.display.show("----")

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
        self.status_topic = cfg["topic"] + "/status"
        self.led_pin = cfg.get("led_pin", None)
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
            t = time.gmtime()
            ts = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(
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
        self.p_min = cfg["p_min"]
        self.p_max = cfg["p_max"]
        self.v_min = cfg.get("i_min", 0.004) * self.r_bocnik
        self.v_max = cfg.get("i_max", 0.020) * self.r_bocnik
        i_disc = cfg.get("i_disconnect", 0.001)
        self.v_disconnect = i_disc * self.r_bocnik if i_disc is not None else None

    def convert_raw(self, raw):
        """Převod raw ADC hodnoty na tlak"""
        voltage = raw * self.shared.v_ref / 32767

        # Odpojený senzor (proud pod prahem — typicky pod 1 mA v 4-20mA smyčce)
        if self.v_disconnect is not None and voltage < self.v_disconnect:
            return None, ERR_NC

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

        if (self.u_divider <= 0 or not 0 <= self.u_min < self.u_max
                or self.p_max <= self.p_min):
            raise ValueError("pirani: u_divider>0, 0<=u_min<u_max, p_min<p_max")

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
    except Exception as e:  # i TypeError — např. číslo v uvozovkách v configu
        print("Chybná konfigurace senzoru:", e)
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
        self._diag_counter = 0
        self._uptime_s = 0

        # LED piny (paralelní list k self.sensors)
        self._leds = []
        for s in self.sensors:
            p = s.led_pin
            self._leds.append(Pin(p, Pin.OUT) if p is not None else None)

        # Tlačítko pro přepínání displeje
        self._btn_last = 0
        self._btn = None
        btn_pin = getattr(config, 'BUTTON_PIN', None)
        if btn_pin is not None and self.sensors:
            self._btn = Pin(btn_pin, Pin.IN, Pin.PULL_UP)
            self._btn.irq(trigger=Pin.IRQ_FALLING, handler=self._btn_irq)

    def _btn_irq(self, pin):
        micropython.schedule(self._btn_pressed, 0)

    def _btn_pressed(self, _):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._btn_last) < 200:
            return
        self._btn_last = now
        self._display_sensor = (self._display_sensor + 1) % len(self.sensors)

    def _publish_diag(self):
        """Publikování diagnostických informací jako JSON"""
        diag = {
            "uptime": self._uptime_s,
            "mem": gc.mem_free(),
            "buf": len(self.shared._pub_buffer),
            "reconn_wifi": self.shared.reconn_wifi,
            "reconn_mqtt": self.shared.reconn_mqtt,
            "ver": VERSION,
        }
        if self.shared.wlan.isconnected():
            try:
                diag["rssi"] = self.shared.wlan.status('rssi')
            except Exception:
                pass
        self.shared.publish(config.DIAG_TOPIC, json.dumps(diag), retain=True)

    def _update_display(self):
        """Zobrazení vybraného senzoru, střídá s chybou připojení"""
        if not self.shared.display or not self.sensors:
            return

        s = self.sensors[self._display_sensor]

        # Pokud je chyba připojení, střídáme hodnotu (2s) a chybu (0.5s)
        if (self._conn_error and s.last_value is not None and not s.last_error
                and time.ticks_ms() % 2500 >= 2000):
            self.shared.show_error(self._conn_error)
            return

        if s.last_error:
            self.shared.show_error(s.last_error)
        elif s.last_value is not None:
            self.shared.show_value(s.last_value)

        for i, led in enumerate(self._leds):
            if led is not None:
                led.value(1 if i == self._display_sensor else 0)

    def run(self):
        
        self.shared.wdt = WDT(timeout=config.WDT_TIMEOUT_MS)

        if not self.shared.connect_mqtt():
            self.shared.show_error(ERR_AUTH if self.shared._mqtt_auth_err else ERR_MQTT)
            self.shared.safe_sleep_ms(2000)

        while True:
            self.shared.wdt.feed()
            self._uptime_s += config.INTERVAL_S
            loop_start = time.ticks_ms()

            t = time.gmtime()
            if t[3] >= 1 and t[2] != self.shared._last_ntp_day:
                if self.shared.wlan.isconnected():
                    self.shared._sync_ntp()
                else:
                    self.shared._last_ntp_day = t[2]

            wifi_ok = self.shared.check_wifi()
            if not wifi_ok:
                self.shared._close_mqtt()
                self.shared.start_wifi_reconnect()

            for s in self.sensors:
                s.read()

            if wifi_ok:
                self.shared.flush_buffer()

            # explicitní smyčka: all(genexp) by po prvním selhání zkratovala
            publish_ok = True
            for s in self.sensors:
                if not s.publish():
                    publish_ok = False

            if wifi_ok and config.DIAG_INTERVAL > 0:
                self._diag_counter += 1
                if self._diag_counter >= config.DIAG_INTERVAL:
                    self._diag_counter = 0
                    self._publish_diag()

            if wifi_ok:
                if self.shared._mqtt_auth_err:
                    self._conn_error = ERR_AUTH
                elif not publish_ok:
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
        # Restart vlastní silou: při výjimce z __init__ ještě WDT neběží (zapíná se
        # v run()), zařízení by jinak zůstalo navěky stát na E--5.
        # 5 s < WDT_TIMEOUT_MS → chování je stejné i s běžícím WDT.
        time.sleep_ms(5000)
        reset()
