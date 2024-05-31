import random
import paho.mqtt.client as mqtt
from controller import Robot
 
robot = Robot()

 
timestep = int(robot.getBasicTimeStep())
 
motor1 = robot.getDevice('motor1')
motor2 = robot.getDevice('motor2')
motor1.setVelocity(0)
motor2.setVelocity(0)



 
motor1.setPosition(float('inf'))  
motor2.setPosition(float('inf'))


broker = '192.168.178.121'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}
 
def MoveForward(speed, duration):
    motor1.setVelocity(speed)
    motor2.setVelocity(speed)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def MoveBack(speed, duration):
    motor1.setVelocity(-speed)
    motor2.setVelocity(-speed)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinLeft(speed, duration):
    motor1.setVelocity(-speed)
    motor2.setVelocity(speed)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinRight(speed, duration):
    motor1.setVelocity(speed)
    motor2.setVelocity(-speed)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinTop(speed, duration):
    motor1.setVelocity(speed)
    motor2.setVelocity(speed)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

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
            print(f"Published '{client_id} connected successfully' to {topics['send']}")
        else:
            print("Invalid configuration format received.")
    else:
        print(f"Received command: {message} " + client_id )
        handle_command(message)

def handle_command(command):
    command = command.strip().upper()
    print(command)
    if command == "FRONT_LED_ON":
        fled.set(True)
    elif command == "FRONT_LED_OFF":
        fled.set(False)
    elif command == "BACK_LED_ON":
        bled.set(True)
    elif command == "BACK_LED_OFF":
        bled.set(False)
    elif command == "MOVE_FORWARD":
        MoveForward(10.0, 1)
    elif command == "MOVE_BACK":
        MoveBack(10.0, 1)
    elif command == "SPIN_LEFT":
        SpinLeft(10.0, 1)
    elif command == "SPIN_RIGHT":
        SpinRight(10.0, 1)
    elif command == "SPIN_TOP":
        SpinTop(10.0, 1)
    else:
        print("Invalid command.")

 
client = mqtt.Client(client_id)
client.on_message = on_message
 
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
        for _ in range(3):
            fled.set(True)
            robot.step(timestep)
            fled.set(False)
            robot.step(timestep)
 

def run_mqtt():
    print("Starting")
    connect_mqtt()
    while robot.step(timestep) != -1:   
        client.loop()

run_mqtt()
