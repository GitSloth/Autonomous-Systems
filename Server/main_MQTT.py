import random
import time
from machine import Pin, PWM, ADC
from paho.mqtt import client as mqtt_client
import network
import socket

broker = 'localhost'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}
counter = 0

BUILT_IN_LED=25 # Built in led
FLED=20 # Front led Red
BLED=21 # Back led Green
PWM_LM=6 # Left Continuous Servo
PWM_RM=7 # Right Continuous Servo
PWM_SC=10 # Panning Servo
SDA=4
SCL=5
MISO=16
MOSI=19
SCK=18
CS=17
LDR_PIN = 27

# Initialize ADC for LDR
ldr = ADC(Pin(LDR_PIN))

# initial state definition
built_in_led = Pin(BUILT_IN_LED, Pin.OUT) # built-in led
fled = Pin(FLED, Pin.OUT) # front led
bled = Pin(BLED, Pin.OUT) # back led
fled.value(True)
bled.value(False) 
built_in_led.value(True)
time.sleep(1)
built_in_led.value(False)
time.sleep(1)
fled.value(False)

# Set up servos
LeftMotor = PWM(Pin(PWM_LM))
LeftMotor.freq(50)
RightMotor = PWM(Pin(PWM_RM))
RightMotor.freq(50)
PanMotor = PWM(Pin(PWM_SC))
PanMotor.freq(50)

# Setup MQTT client
client = mqtt_client.Client(client_id)

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/commands")
        print(f"Subscribed to robots/{client_id}/commands")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    command = msg.payload.decode()
    print(f"Received command: {command}")
    handle_command(command)

def on_disconnect(client, userdata, rc):
    print("Disconnected from MQTT Broker!")

# Handle MQTT commands
def handle_command(command):
    if command == "FRONT_LED_ON":
        fled.value(True)
    elif command == "FRONT_LED_OFF":
        fled.value(False)
    elif command == "BACK_LED_ON":
        bled.value(True)
    elif command == "BACK_LED_OFF":
        bled.value(False)
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

# Connect to MQTT broker and start listening for commands
def connect_mqtt():
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(broker, port)
    return client

# MQTT client loop
def run_mqtt():
    global client
    client = connect_mqtt()
    client.loop_forever()

#=========================MOVES============================
# Function controlling servos
def MoveForward(power, Stime):
    LeftMotor.duty_u16(2000)
    RightMotor.duty_u16(6600)
    time.sleep(Stime)
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)
    SpinTop()

def MoveBack(power, Stime):
    LeftMotor.duty_u16(6600)
    RightMotor.duty_u16(2000)
    time.sleep(Stime)
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)
    SpinTop()

def SpinLeft(power, Stime):
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(6000)
    time.sleep(Stime)
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)
    SpinTop()

def SpinRight(power, Stime):
    LeftMotor.duty_u16(2000)
    RightMotor.duty_u16(5000)
    time.sleep(Stime)
    LeftMotor.duty_u16(5000)
    RightMotor.duty_u16(5000)
    SpinTop()

def SpinTop(power, Stime):
    duty_cycles = [2000, 3000, 4000, 5000, 6000]
    for duty in duty_cycles:
        PanMotor.duty_u16(duty)
        time.sleep(0.5)
        print("LDR Value:", ldr.read_u16())
        time.sleep(1)

    PanMotor.duty_u16(5000)
    print("LDR Value:", ldr.read_u16())
    time.sleep(1)

#==============================WIFI=============================
# Activate the Pico Lan
network.hostname("mypicow")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Hostname set to: " + str(network.hostname()))

time0 = time.time()
wlan.connect(ssid, pwd)
while True:
    if wlan.isconnected():
        print("\nConnected!\n")
        built_in_led.value(True)
        break
    else:
        print(".")
        time.sleep(1)
        if time.time() - time0 > 10:
            print("Connection could not be established")
            break

sta_if = network.WLAN(network.STA_IF)
print(sta_if.ifconfig()[0])  # Print the IP on the serial

# Listen for MQTT commands
run_mqtt()
