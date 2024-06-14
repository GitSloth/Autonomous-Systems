import random
import time
from machine import Pin, PWM, ADC
from umqtt.simple import MQTTClient
import network
import math
import json
import _thread

# pins
BUILT_IN_LED = 25  # Built-in LED
FLED = 20  # Front LED Red
BLED = 21  # Back LED Green
PWM_LM = 6  # Left Continuous Servo
PWM_RM = 7  # Right Continuous Servo
PWM_SC = 10  # Panning Servo
LDR_PIN = 27

# sensor readings
ldr_readings = []
distance_readings = []
radius = 120
robot_data = {}

# Initialize ADC for LDR
ldr = ADC(Pin(LDR_PIN))

# Initialise LEDs
built_in_led = Pin(BUILT_IN_LED, Pin.OUT)
fled = Pin(FLED, Pin.OUT)
bled = Pin(BLED, Pin.OUT)
fled.value(True)
bled.value(True)
built_in_led.value(True)
time.sleep(1)
built_in_led.value(False)
time.sleep(1)
fled.value(False)
bled.value(False)

# Set up servos
LeftMotor = PWM(Pin(PWM_LM))
LeftMotor.freq(50)
RightMotor = PWM(Pin(PWM_RM))
RightMotor.freq(50)
PanMotor = PWM(Pin(PWM_SC))
PanMotor.freq(50)
stop_duty = 4915
duty_range = 1638

left_velocity = 20
right_velocity = 30

# MQTT settings
broker = '10.198.64.131'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {'send': None, 'receive': None}
counter = 0
subscribed_topics = set()
client = MQTTClient(client_id, broker, port)
# Event to signal position update
position_update_flag = False


# Set servo speed of the left motor
def set_servo_speed_left(speed):
    duty = stop_duty + int(speed * duty_range / 100)  
    LeftMotor.duty_u16(duty)

# Set servo speed of the right motor
def set_servo_speed_right(speed):
    duty = stop_duty + int((speed * (-1)) * duty_range / 100)  
    RightMotor.duty_u16(duty)

# Forward
def MoveForward(duration):
    global left_velocity
    global right_velocity
    print("MoveForward called with duration:", duration)
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def MoveBack(duration):
    print("MoveBack called with duration:", duration)
    set_servo_speed_left(-left_velocity)
    set_servo_speed_right(-right_velocity)
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinLeft(duration):
    set_servo_speed_left(0)
    set_servo_speed_right(right_velocity)
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinRight(duration):
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(0)
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinTop():
    duty_cycles = [2000, 3000, 4000, 5000, 6000, 7000, 8000]
    for duty in duty_cycles:
        PanMotor.duty_u16(duty)
        time.sleep(0.1)
        print("LDR Value:", ldr.read_u16())
        time.sleep(0.1)

    PanMotor.duty_u16(5000)
    print("LDR Value:", ldr.read_u16())
    time.sleep(1)

def MoveForwardCont():
    print("MoveForwardCont called")
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)

def Stop():
    print("Stop")
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def reboot():
    print('Rebooting...')
    machine.reset()

def calculate_intersection_points(coord1, coord2, radius):
    """Calculate the intersection points of two circles."""
    d = math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)
    if d > 2 * radius or d == 0:
        return None
    
    a = (radius**2 - radius**2 + d**2) / (2 * d)
    h = math.sqrt(radius**2 - a**2)
    
    x2 = coord1[0] + a * (coord2[0] - coord1[0]) / d
    y2 = coord1[1] + a * (coord2[1] - coord1[1]) / d
    
    x3 = x2 + h * (coord2[1] - coord1[1]) / d
    y3 = y2 - h * (coord2[0] - coord1[0]) / d
    
    x4 = x2 - h * (coord2[1] - coord1[1]) / d
    y4 = y2 + h * (coord2[0] - coord1[0]) / d
    
    midpoint_x = (x3 + x4) / 2
    midpoint_y = (y3 + y4) / 2
    return (midpoint_x, midpoint_y)

def check_intersections(current_position, radius):
    global robot_data
    global client_id
    intersections = []
    for robot_id, info in robot_data.items():
        if robot_id == client_id:
            continue
        other_position = info['position']
        points = calculate_intersection_points(current_position, other_position, radius)
        if points:
            intersections.append(points)
    return intersections

def check_border_intersection(current_position, radius, width, height):
    x, y = current_position
    radius = radius
    intersections = []
    if x - radius < 20:
        intersections.append((0, y))
    if x + radius > width - 20:
        intersections.append((width, y))
    if y - radius < 20:
        intersections.append((x, 0))
    if y + radius > height - 20:
        intersections.append((x, height))
    return intersections

def avoid_collisions(current_position, current_angle, intersections, border_intersections, radius, width, height):
    global robot_data  
    
    print("Avoiding")
    
    if border_intersections:
        for border in border_intersections:
            border_angle = calculate_relative_angle(current_position, current_angle, border)
            if 0 <= border_angle <= 90:
                SpinLeft(0.3)
                return
            elif 90 < border_angle < 270: 
                MoveForward(0.3)
                return
            elif 270 <= border_angle <= 360:
                SpinRight(0.3) 
                return
    
    for point in intersections:
        angle = calculate_relative_angle(current_position, current_angle, point)
        print(angle)
        if 0 < angle <= 90:
            print("avoid to the left")
            SpinLeft(0.3)
            return
        elif 90 < angle <= 270:
            print("avoid forward")
            MoveForward(0.3)
            return
        elif 270 < angle <= 360:
            print("avoid to the right")
            SpinRight(0.3)
            return
    
    print("No collisions found?")

def calculate_relative_angle(current_position, current_angle, point):
    dx = point[0] - current_position[0]
    dy = point[1] - current_position[1]
    angle_to_point = math.atan2(dy, dx) * 180 / math.pi  # Convert radians to degrees
    relative_angle = (angle_to_point - current_angle) % 360
    return relative_angle

def pathing_light():
    global robot_data
    global client_id
    
    client.publish(topics['send'], b"request_positions")
    client.loop()
    print("Requesting position updates...")
    start_time = time.time()
    timeout = 1
    # Wait for the position update event
    while not position_update_flag:
        
        if start_time + timeout < time.time():
            print("Timeout waiting for position updates.")
            return
    position_update_flag = False
    try:
        current_position = robot_data[client_id]['position']
        current_angle = robot_data[client_id]['angle']
    except KeyError:
        print(f"No key with that name: {client_id}")
        return

    intersections = check_intersections(current_position, radius)
    border_intersections = check_border_intersection(current_position, radius, 1280, 720)

    if intersections or border_intersections:
        avoid_collisions(current_position, current_angle, intersections, border_intersections, radius, 1200, 600)
    else:
        MoveForwardCont()

# MQTT callbacks
def on_message(topic, msg):
    print("I love cheese")
    global topics
    global robot_data
    global position_update_flag
    topic = topic.decode('utf-8')
    message = msg.decode('utf-8')

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
    elif topic == topics['receive']:
        print("Received message on the receive topic.")
        try:
            data = json.loads(message)
            for robot_id, robot_info in data.items():
                if robot_id != client_id:
                    if robot_id in robot_data:
                        robot_data[robot_id]['position'] = robot_info['position']
                        robot_data[robot_id]['angle'] = robot_info['angle']
                    else:
                        robot_data[robot_id] = {'position': robot_info['position'], 'angle': robot_info['angle']}
                    print(f"Updated robot '{robot_id}' position: {robot_data[robot_id]['position']}, angle: {robot_data[robot_id]['angle']}")
                else:
                    print(f"Current robot: {client_id} position: {robot_info['position']}, angle: {robot_info['angle']}")
                    robot_data[robot_id] = {'position': robot_info['position'], 'angle': robot_info['angle']}
            
            print("Updated robot data:", robot_data)
            # Signal that the position update has been received
            position_update_flag = True
        except ValueError:
            print(f"Received command: {message} on {topic}")
            handle_command(message)

def handle_command(command):
    command = command.strip().upper()
    print(command)
    if command == "FRONT_LED_ON":
        fled.value(True)
    elif command == "FRONT_LED_OFF":
        fled.value(False)
    elif command == "BACK_LED_ON":
        bled.value(True)
    elif command == "BACK_LED_OFF":
        bled.value(False)
    elif command == "MOVE_FORWARD":
        MoveForward(1)
    elif command == "MOVE_FORWARD_CONT":
        MoveForwardCont()
    elif command == "MOVE_BACK":
        MoveBack(1)
    elif command == "SPIN_LEFT":
        SpinLeft(1)
    elif command == "SPIN_RIGHT":
        SpinRight(1)
    elif command == "SPIN_TOP":
        SpinTop(10.0, 1)
    elif command == "STOP":
        Stop()
    elif command == "REBOOT":
        reboot()
    else:
        print("Invalid command.")

def connect_mqtt():
    global client
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
        for i in range(3):
            fled.value(True)
            time.sleep(0.2)
            fled.value(False)
            time.sleep(0.2)

def run_mqtt():
    global client
    connect_mqtt()
    while True:
        try:
            client.check_msg()
        except OSError as e:
            print(f"Error in MQTT loop: {e}")
            time.sleep(2)
            connect_mqtt()
        time.sleep(0.01)

#==============================WIFI=============================
ssid = 'ssid'
password = 'pass'

network.hostname(client_id)
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
print(sta_if.ifconfig()[0])

# Start MQTT in a separate thread
_thread.start_new_thread(run_mqtt, ())
time.sleep(1)
# Main loop for pathing and periodic tasks
last_pathing_time = 0
pathing_interval = 0.5  # Time interval in seconds

while True:
    current_time = time.time()
    
    if current_time - last_pathing_time >= pathing_interval:
        pathing_light()
        last_pathing_time = current_time
    
    
    time.sleep(0.01)
