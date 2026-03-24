#!/usr/bin/env python3
"""MQTT → PostgreSQL daemon.

Subscribes to MQTT sensor topics and inserts float values into sensor_value
using the same INSERT...SELECT pattern as postgres_query.c.
"""

import collections
import configparser
import logging
import signal
import sys
import threading

import paho.mqtt.client as mqtt
import psycopg2

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
log = logging.getLogger(__name__)

VERSION = "1.0.2"
QUEUE_MAX = 10_000
#INSERT_SQL = (
#    "INSERT INTO sensor_value (sensor_id, value, create_tms) "
#    "SELECT id, %s, %s FROM sensor WHERE name = %s"
#)
INSERT_SQL = "SELECT insert_sensor_value(%s, %s, %s)"

def load_config(path):
    cfg = configparser.ConfigParser()
    if not cfg.read(path):
        log.error("Config file not found: %s", path)
        sys.exit(1)
    return cfg


def db_worker(conn_string, insert_queue, stop_event):
    conn = None
    backoff = 1

    while not stop_event.is_set() or insert_queue:
        # Connect / reconnect
        if conn is None:
            try:
                conn = psycopg2.connect(conn_string)
                conn.autocommit = True
                log.info("Connected to PostgreSQL")
                backoff = 1
            except Exception as e:
                log.warning("DB connect failed: %s — retry in %ds", e, backoff)
                stop_event.wait(backoff)
                backoff = min(backoff * 2, 60)
                continue

        # Flush queue in batches
        if insert_queue:
            batch = []
            try:
                while insert_queue and len(batch) < 500:
                    batch.append(insert_queue.popleft())
            except IndexError:
                pass

            try:
                with conn.cursor() as cur:
                    cur.executemany(INSERT_SQL, batch)
                log.debug("Inserted %d rows", len(batch))
            except Exception as e:
                log.error("DB insert failed: %s", e)
                # Return items to queue (prepend to preserve order)
                insert_queue.extendleft(reversed(batch))
                try:
                    conn.close()
                except Exception:
                    pass
                conn = None
                stop_event.wait(backoff)
                backoff = min(backoff * 2, 60)
        else:
            stop_event.wait(0.5)

    if conn:
        conn.close()
        log.info("DB connection closed")


def main():
    cfg_path = "mqtt_push.cfg"
    if len(sys.argv) > 1:
        cfg_path = sys.argv[1]

    log.info("Starting mqtt_to_postgres v%s", VERSION)

    cfg = load_config(cfg_path)
    conn_string = cfg["postgres"]["connection"]
    broker = cfg["mqtt"]["broker"]
    port = cfg["mqtt"].getint("port", 1883)
    topic = cfg["mqtt"]["topic"]
    strip_prefix = cfg["sensor"].get("strip_prefix", "sensor/")

    insert_queue = collections.deque(maxlen=QUEUE_MAX)
    stop_event = threading.Event()

    def on_message(client, userdata, msg):
        t = msg.topic
        if t.endswith("/status"):
            return
        try:
            parts = msg.payload.decode().split()
            value = float(parts[0])
            ts = parts[1]
        except (ValueError, TypeError, IndexError):
            return
        sensor_name = t.removeprefix(strip_prefix)
#        insert_queue.append((value, ts, sensor_name))
        insert_queue.append((sensor_name, value, ts))

    def on_connect(client, userdata, flags, rc, *args):
        if rc == 0:
            log.info("MQTT connected, subscribing to %s", topic)
            client.subscribe(topic)
        else:
            log.warning("MQTT connect failed, rc=%d", rc)

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            log.warning("MQTT unexpected disconnect rc=%d", rc)

    worker = threading.Thread(
        target=db_worker, args=(conn_string, insert_queue, stop_event), daemon=True
    )
    worker.start()

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(broker, port, keepalive=60)

    def shutdown(signum, frame):
        log.info("Shutting down...")
        client.disconnect()
        client.loop_stop()
        stop_event.set()
        worker.join(timeout=10)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    client.loop_forever()


if __name__ == "__main__":
    main()
