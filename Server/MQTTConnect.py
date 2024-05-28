import time
from paho.mqtt import client as mqtt_client
# connecting to the server 
broker = 'localhost'
port = 1883
client_id = 'server'

#define on connect
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Server connected to MQTT Broker!")
        client.subscribe("swarm/register")
    else:
        print("Failed to connect, return code", rc)

def on_message(client, userdata, msg):
    print(f"printers")
    global next_robot_id
    if msg.topic == "swarm/register":
        robot_id = msg.payload.decode()
        topic_receive = f"robots/{robot_id}/receive"
        topic_send = f"robots/{robot_id}/send"
        client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        print(f"Assigned ID {robot_id} to new robot. Config sent.")

        #client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        print(f"Created topics for robot {robot_id}: {topic_receive}, {topic_send}")

def connect_mqtt():
    
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    return client

def run():
    client = connect_mqtt()
    client.loop_start()
    while True:
        time.sleep(1)

if __name__ == '__main__':
    run()
