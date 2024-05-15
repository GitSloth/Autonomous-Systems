import logging
import random
import time
from paho.mqtt import client as mqtt_client

broker = 'localhost'
port = 1883
topic_server = "test/test"
topic_robots = "test/test"
client_id = f'python-mqtt-{random.randint(0, 1000)}'


client = None

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
FLAG_EXIT = False

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(topic_robots)
    else:
        print("Failed to connect, return code", rc)

def on_message(client, userdata, msg):
    if msg.topic == "test/test":
        print("Hello")

def on_disconnect(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)
    global FLAG_EXIT
    FLAG_EXIT = True

def connect_mqtt():
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(broker, port)
    return client

def publish(client):
   
    pass

def run():
    global client
    client = connect_mqtt()
    client.loop_start()
    time.sleep(1)
    timer = 0
   
    while not FLAG_EXIT:
        if timer == 960:
            publish(client)
            timer = 0
        timer += 64
        pass

if __name__ == '__main__':
    run()
