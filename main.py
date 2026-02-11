#
# Device: RPI Pico 2W
# Podpora více heterogenních senzorů na ADS1115
#
from machine import I2C, Pin, WDT
from ads1x15 import ADS1115
import tm1637
from umqtt.simple import MQTTClient
import network
import time
import math
import config

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


class SharedResources:
    """Sdílené zdroje pro všechny senzory"""

    def __init__(self):
        self.wdt = None
        self.v_ref = GAIN_VREF.get(config.ADC_GAIN)

        # Display jako první - pro zobrazení chyb při inicializaci
        try:
            self.display = tm1637.TM1637Decimal(clk=Pin(config.TM_CLK),
                                                dio=Pin(config.TM_DIO))
            self.display.brightness(7)
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
        if not self.wlan.isconnected():
            self.wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)

        for _ in range(20):  # 20 × 0.5s = max 10s
            if self.wlan.isconnected():
                break
            self.safe_sleep_ms(500)

        if self.wlan.isconnected():
            print("WiFi OK:", self.wlan.ifconfig()[0])
            return True
        else:
            print("WiFi CHYBA")
            return False

    def check_wifi(self):
        """Kontrola Wi-Fi připojení"""
        return self.wlan.isconnected()

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
        total = 0
        count = 0
        for i in range(samples):
            try:
                raw = self.adc.read(config.ADC_RATE, channel)
                total += raw
                count += 1
            except OSError as e:
                print("ADC error:", e)
                self.adc = None
                break
            if i % 10 == 9 and self.wdt:
                self.wdt.feed()
        if count == 0:
            return None, ERR_ADC
        return total // count, None

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

        try:
            self.mqtt = MQTTClient(config.MQTT_CLIENT_ID,
                                   config.MQTT_BROKER,
                                   config.MQTT_PORT,
                                   keepalive=config.MQTT_KEEPALIVE)
            self.mqtt.set_last_will(config.MQTT_STATUS_TOPIC,
                                    "offline", retain=True)
            self.mqtt.connect()
            self.mqtt.publish(config.MQTT_STATUS_TOPIC, "online", retain=True)
            self._mqtt_backoff = 1000
            self._mqtt_last_ping = time.ticks_ms()
            return True
        except OSError as e:
            print("MQTT error:", e)
            self._close_mqtt()
            self._mqtt_next_try = time.ticks_add(now, self._mqtt_backoff)
            self._mqtt_backoff = min(self._mqtt_backoff * 2, 60_000)
            return False

    def mqtt_ping(self):
        """Periodický MQTT ping pro udržení spojení"""
        if self.mqtt is None:
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, self._mqtt_last_ping) < config.MQTT_KEEPALIVE * 500:
            return
        try:
            self.mqtt.ping()
            self._mqtt_last_ping = now
        except OSError:
            self._close_mqtt()

    def publish(self, topic, payload, retain=True):
        """Odeslání payload na MQTT topic"""
        if not self.connect_mqtt():
            return False
        try:
            self.mqtt.publish(topic, str(payload), retain=retain)
            return True
        except OSError:
            self._close_mqtt()
            return False

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
        self.last_value = value
        self.last_error = error
        return value, error

    def publish(self):
        """Publikování hodnoty a stavu na oddělené topicy"""
        ok = True

        # Stav: "OK" nebo srozumitelná chybová zpráva pro MQTT
        if self.last_error:
            status = ERR_MQTT_MSG.get(self.last_error, self.last_error)
        else:
            status = "OK"
        if not self.shared.publish(self.status_topic, status):
            ok = False

        # Hodnotu publikuj jen když je platná
        if self.last_value is not None:
            formatted = "{:.{}f}".format(self.last_value, self.precision)
            if not self.shared.publish(self.topic, formatted):
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
        self.u_min = cfg.get("u_min", 0.0)
        self.u_max = cfg.get("u_max", shared.v_ref or 1.024)
        self.p_min = cfg.get("p_min", 1e-4)
        self.p_max = cfg.get("p_max", 1000.0)

    def convert_raw(self, raw):
        """Převod raw ADC hodnoty na tlak přes regresní model"""
        voltage = raw * self.shared.v_ref / 32767

        if voltage < self.u_min:
            return None, ERR_LO
        if voltage > self.u_max:
            return None, ERR_HI

        try:
            pressure = math.exp(self.a + self.b * voltage + self.c * math.sqrt(voltage))
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

            wifi_ok = self.shared.check_wifi()
            if not wifi_ok:
                self.shared._close_mqtt()
                self.shared.connect_wifi()
                wifi_ok = self.shared.check_wifi()

            self._read_all()

            if wifi_ok:
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
