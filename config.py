# Wi-Fi
WIFI_SSID = "tvoje_SSID"
WIFI_PASSWORD = "tvoje_heslo"

# MQTT
MQTT_BROKER = "192.168.1.x"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "pico_L200h"
MQTT_KEEPALIVE = 60          # [s] keepalive interval pro broker
MQTT_STATUS_TOPIC = "sensor/L200h/status"  # LWT a online status

# I2C piny
I2C_ID = 0
I2C_SDA = 0
I2C_SCL = 1
I2C_FREQ = 400_000

# Display
TM_CLK = 16
TM_DIO = 17
DISPLAY_SENSOR = 0  # index senzoru pro zobrazení na displeji (0 = první)

# Sdílené ADC nastavení
ADC_ADDRESS = 0x48  # I2C adresa ADS1115 (0x48-0x4B podle ADDR pinu)
ADC_GAIN = 4        # ±1.024 V
ADC_RATE = 3        # 64 SPS
ADC_SAMPLES = 5     # počet vzorků pro medián (volit liché číslo!)

# Interval měření [s]
INTERVAL_S = 1

# Watchdog timeout [ms]
WDT_TIMEOUT_MS = 8000

# Seznam senzorů s per-senzor konfigurací
SENSORS = [
    {
        "type": "current_loop",      # typ převodu
        "channel": 0,                # ADC kanál
        "topic": "sensor/L200h/p1" , # MQTT topic
        "name": "p1",                # pro debug
        # Parametry specifické pro current_loop:
        "r_bocnik": 50,  # [Ohm]
        "i_min": 0.004,  # [A]
        "i_max": 0.020,  # [A]
        "p_min": 0,      # [kPa]
        "p_max": 120,    # [kPa]
        "precision": 3,  # počet desetinných míst pro MQTT
    },
    # Příklad Pirani vakuometru (odkomentovat a upravit):
    # {
    #     "type": "pirani",            # Pirani vakuometr
    #     "channel": 1,
    #     "topic": "sensor/L200h/pirani",
    #     "name": "Pirani",
    #     # Regresní koeficienty: p = exp(a + b*u + c*sqrt(u)) [mbar]
    #     "a": -6.435,
    #     "b": 7.418,
    #     "c": 2.536,
    #     "u_min": 0.0,    # [V] min. napětí (pod = ERR_LO)
    #     "u_max": 1.024,  # [V] max. napětí (nad = ERR_HI)
    #     "p_min": 1e-4,   # [mbar] min. tlak (pod = ERR_LO)
    #     "p_max": 1000.0, # [mbar] max. tlak (nad = ERR_HI)
    #     "precision": 3,  # počet desetinných míst pro MQTT
    # },
]
