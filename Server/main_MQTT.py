import random
import time
from machine import Pin, PWM, ADC
from umqtt.simple import MQTTClient
import network

broker = '10.198.64.131'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}  # Ensure topics is an empty dictionary initially
counter = 0
subscribed_topics = set()

BUILT_IN_LED = 25  # Built-in LED
FLED = 20  # Front LED Red
BLED = 21  # Back LED Green
PWM_LM = 6  # Left Continuous Servo
PWM_RM = 7  # Right Continuous Servo
PWM_SC = 10  # Panning Servo
LDR_PIN = 27

# Initialize ADC for LDR
ldr = ADC(Pin(LDR_PIN))

# Initial state definition
built_in_led = Pin(BUILT_IN_LED, Pin.OUT)  # Built-in LED
fled = Pin(FLED, Pin.OUT)  # Front LED
bled = Pin(BLED, Pin.OUT)  # Back LED
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
client = MQTTClient(client_id, broker, port)

# MQTT callbacks
def on_message(topic, msg):
    global topics
    if topic.decode() == f"robots/{client_id}/config":
        print("Received configuration response.")
        config = msg.decode().split(',')
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
        command = msg.decode()
        print(f"Received command: {command}")
        handle_command(command)
        
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
        print(f"hoo")
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
    try:
        client.set_callback(on_message)
        client.connect()
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/config")
        print(f"Subscribed to robots/{client_id}/config")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    except Exception as e:
        print(f"Failed to connect to MQTT Broker: {e}")
        fled.value(True)
        time.sleep(0.2)
        fled.value(False)
        time.sleep(0.2)
        fled.value(True)
        time.sleep(0.2)
        fled.value(False)
        time.sleep(0.2)
        fled.value(True)
        time.sleep(0.2)
        fled.value(False)

# MQTT client loop
def run_mqtt():
    global client
    connect_mqtt()
    while True:
        try:
            client.check_msg()  # Check for new messages
        except OSError as e:
            print(f"Error in MQTT loop: {e}")
            time.sleep(2)  # Wait before retrying
            connect_mqtt()
        time.sleep(0.01)
#=========================MOVES============================
# Function controlling servos
def MoveForward(power, Stime):
    print(f"hiii")
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

def SpinTop(power=50, Stime=1):
    duty_cycles = [2000, 3000, 4000, 5000, 6000, 7000, 8000]
    for duty in duty_cycles:
        PanMotor.duty_u16(duty)
        time.sleep(0.1)
        print("LDR Value:", ldr.read_u16())
        time.sleep(0.1)

    PanMotor.duty_u16(5000)
    print("LDR Value:", ldr.read_u16())
    time.sleep(1)

#==============================WIFI=============================
# Activate the Pico LAN
ssid = 'ssid'
password = 'pass'

network.hostname("mypicow")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Hostname set to: " + str(network.hostname()))

time0 = time.time()
wlan.connect(ssid, password)
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


