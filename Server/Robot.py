import random
import time
from paho.mqtt import client as mqtt_client

broker = 'localhost'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}
counter = 0

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/config")
        # client.subscribe(f"robots/{client_id}/send")
        # client.subscribe(f"robots/{client_id}/receive")
        print(f"Subscribed to robots/{client_id}/config")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    global topics
    if msg.topic == f"robots/{client_id}/config":
        print("Received configuration response.")
        config = msg.payload.decode().split(',')
        if len(config) == 2:
            topics['receive'] = config[0]
            topics['send'] = config[1]
            client.subscribe(topics['receive'])
            print(f"Subscribed to {topics['receive']}")
            client.publish(topics['send'], client_id + "connected succesfully")
            print(f"Published 'bababoei' to {topics['send']}")
        else:
            print("Invalid configuration format received.")
    else:
        print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")

def on_disconnect(client, userdata, rc):
    print("Disconnected from MQTT Broker!")

def connect_mqtt():
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(broker, port)
    return client


def run():
    global client
    client = connect_mqtt()
    client.loop_start()  # Start the MQTT client loop

    global counter
    # Keep publishing the message every 4 seconds
    while True:
        time.sleep(1)
        if client.is_connected():
            counter += 1
            message = f"{client_id} connected successfully, count: {counter}"
            client.publish(topics['send'], message)
            print(f"Published '{message}' to {topics['send']}")

if __name__ == '__main__':
    run()
