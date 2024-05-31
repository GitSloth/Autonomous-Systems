import random
import time
import paho.mqtt.client as mqtt

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize LEDs or other actuators if not done elsewhere in your code
fled = robot.getDevice("front_led")
bled = robot.getDevice("back_led")

# Define movement functions
def MoveForward(speed, duration):
    pass  # Implement according to your robot's motion control

def MoveBack(speed, duration):
    pass  # Implement according to your robot's motion control

def SpinLeft(speed, duration):
    pass  # Implement according to your robot's motion control

def SpinRight(speed, duration):
    pass  # Implement according to your robot's motion control

def SpinTop(speed, duration):
    pass  # Implement according to your robot's motion control

# MQTT setup
broker = '192.168.178.121'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}


# MQTT callbacks
def on_message(client, userdata, msg):
    global topics
    topic = msg.topic
    message = msg.payload.decode()
    
    if topic == f"robots/{client_id}/config":
        print("Received configuration response.")
        config = message.split(',')
        if len(config) == 2:
            topics['receive'] = config[0]
            topics['send'] = config[1]
            client.subscribe(topics['receive'])
            print(f"Subscribed to {topics['receive']}")
            client.publish(topics['send'], f"{client_id} connected successfully")
            print(f"Published 'bababoei' to {topics['send']}")
        else:
            print("Invalid configuration format received.")
    else:
        print(f"Received command: {message}")
        handle_command(message)

def handle_command(command):
    command = command.strip().upper()
    print(command)
    if command == "FRONT_LED_ON":
        fled.setValue(True)
    elif command == "FRONT_LED_OFF":
        fled.setValue(False)
    elif command == "BACK_LED_ON":
        bled.setValue(True)
    elif command == "BACK_LED_OFF":
        bled.setValue(False)
    elif command == "MOVE_FORWARD":
        MoveForward(50, 1)
    elif command == "MOVE_BACK":
        MoveBack(50, 1)
    elif command == "SPIN_LEFT":
        SpinLeft(50, 1)
    elif command == "SPIN_RIGHT":
        SpinRight(50, 1)
    elif command == "SPIN_TOP":
        SpinTop(50, 1)
    else:
        print("Invalid command.")


# Setup MQTT client
client = mqtt.Client(client_id)
client.on_message = on_message
# Connect to MQTT broker and start listening for commands
def connect_mqtt():
    try:
        client.connect(broker, port)
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/config")
        print(f"Subscribed to robots/{client_id}/config")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    except Exception as e:
        print(f"Failed to connect to MQTT Broker: {e}")
        for i in range(3):
            fled.setValue(True)
            time.sleep(0.2)
            fled.setValue(False)
            time.sleep(0.2)

# MQTT client loop
def run_mqtt():
    print("starting")
    connect_mqtt()
    client.loop_forever()

run_mqtt()
