import random
import time
from machine import Pin, PWM, ADC, I2C
from vl53l0x import VL53L0X
from umqtt.simple import MQTTClient
import network
import math
import json

def read_settings():
    try:
        with open("settings.json", "r") as f:
            settings = json.load(f)
            print("succesfully import settings")
        return settings
    except Exception as e:
        print("Failed to read settings file:", e)
        return None
    
settings = read_settings()
if settings is None:
    print("failed to get settings.")
    exit()


# Pins
BUILT_IN_LED = 25  # Built-in LED
FLED = 20  # Front LED Red
BLED = 21  # Back LED Green
PWM_LM = 6  # Left Continuous Servo
PWM_RM = 7  # Right Continuous Servo
PWM_SC = 10  # Panning Servo
LDR_PIN_ID = 27 # ldr sensor
sda_pin = Pin(0) # SDA pin
scl_pin = Pin(1) # SCL pin

# Sensors
# ldr
ldr_pin = Pin(LDR_PIN_ID, mode=Pin.IN, value=None,pull=None)
ldr = ADC(ldr_pin)
ldr_readings = []
last_spin_time = 0
ldr_min_threshold = settings["ldr_min"]
ldr_max_threshold = settings["ldr_max"]  

# distance sensor
distance_readings = []
i2c = I2C(0, sda=sda_pin, scl=scl_pin, freq=400000)
devices = i2c.scan()
print("I2C devices found:", devices)
distance_sensor = VL53L0X(i2c)

# Initialise leds
built_in_led = Pin(BUILT_IN_LED, Pin.OUT)  # Built-in LED
fled = Pin(FLED, Pin.OUT)  # Front LED
bled = Pin(BLED, Pin.OUT)  # Back LED
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

# Speed
left_velocity = settings["left_velocity"]
right_velocity = settings["right_velocity"]

# mqtt settings
broker = settings["mqtt_broker"] # change ip based on network 
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
client = MQTTClient(client_id, broker, port)
topic_register = "swarm/register"
topics = {}
counter = 0
subscribed_topics = set()
update_interval = 50  # Update interval in milliseconds

notfinished = True
target_position = None

target_tolerance = 170

# positioning
radius = 80
border_buffer = 40
robot_data = {}
pos_updated = False
started = False
# wifi
ssid = settings["wifi_ssid"]
password = settings["wifi_password"]

#========================Dijkstra
class Graph:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.edges = {}

    def neighbors(self, id):
        x, y = id
        results = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        results = filter(self.in_bounds, results)
        return results

    def in_bounds(self, id):
        x, y = id
        return 0 <= x < self.width and 0 <= y < self.height

graph = Graph(128, 72)  # Assuming a 128x72 grid

import heapq

def dijkstra(graph, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not len(frontier) == 0:
        current = heapq.heappop(frontier)[1]

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + 1  # Assume each move costs 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                heapq.heappush(frontier, (priority, next))
                came_from[next] = current

    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)  # optional
    path.reverse()  # optional
    return path

#=========================MOVES============================
#set servo speed of the left motor
#-100 to 100, 0 for stop
def set_servo_speed_left(speed):
    #if speed > 100:
    #    speed = 100
    #elif speed < -100:
    #    speed = -100
     
    duty = stop_duty + int(speed * duty_range / 100)  
    LeftMotor.duty_u16(duty)        

#set servo speed of the right motor
#-100 to 100, 0 for stop
def set_servo_speed_right(speed):
    #if speed > 100:
    #    speed = 100
    #elif speed < -100:
    #    speed = -100
    duty = stop_duty + int((speed * (-1)) * duty_range / 100)  
    RightMotor.duty_u16(duty)
    
# forward  
def MoveForward(duration):
    global left_velocity
    global right_velocity
    print("MoveForward called with duration:", duration)
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def MoveBack(duration):
    global left_velocity
    global right_velocity
    print("MoveBack called with duration:", duration)
    set_servo_speed_left(-left_velocity)
    set_servo_speed_right(-right_velocity)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinLeft(duration):
    global left_velocity
    global right_velocity
    set_servo_speed_left(0)
    set_servo_speed_right(right_velocity)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinRight(duration):
    global left_velocity
    global right_velocity
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(0)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)



def MoveForwardCont():
    global left_velocity
    global right_velocity
    print("MoveForwardCont called")
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)

def Stop():
    print("Stop")
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinTop(speed, duration):
    Stop()
    ldr_readings = []
    distance_readings = []
    angles = []
    steps = 10
    duty_min = settings["duty_0"]  # all the way right
    duty_mid = settings["duty_mid"] # middle
    duty_max = settings["duty_180"]  # all the way left
    angle_min = 0
    angle_max = 180
    
    step_size = (duty_max - duty_min) // (steps-1)
    #duty_cycles = [2000, 3000, 4000, 5000, 6000, 7000, 8000]
    for i in range(steps):
        duty = duty_min + step_size * i
        angle = angle_min + (angle_max - angle_min) * (duty - duty_min) / (duty_max - duty_min)
        PanMotor.duty_u16(duty)
        end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
        while time.ticks_diff(end_time, time.ticks_ms()) > 0:
            pass
        first_reading = ldr.read_u16()
        second_reading = ldr.read_u16()
        difference = abs(first_reading - second_reading)
        print(f"ldr dif: {difference}")
        while not (difference < 1000):
            first_reading = ldr.read_u16()
            second_reading = ldr.read_u16()
            difference = abs(first_reading - second_reading)
            print(f"ldr dif: {difference}")
        ldr_readings.append(first_reading)
        try:
            distance_value = distance_sensor.read()
        except Exception as e:
            print(e)
            distance_value = 999
        distance_readings.append(distance_value)
        angles.append(angle)
    PanMotor.duty_u16(duty_mid)
    distance_sensor.stop()
    return ldr_readings, distance_readings, angles

def reboot():
    print('Rebooting...')
    machine.reset()

def calculate_intersection_points(coord1, coord2, radius):
    """Calculate the intersection points of two bots given their positions and direction vectors."""
    d = math.sqrt((coord2[0] - coord1[0]) ** 2 + (coord2[1] - coord1[1]) ** 2)
    
    # No intersection if distance is greater than 2 times the radius or zero
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

def check_intersections(current_position, current_vector, radius):
    """Get the points of intersection with other bots."""
    global robot_data
    global client_id
    intersections = []
    for robot_id, info in robot_data.items():
        if robot_id == client_id:
            continue
        other_position = info['position']
        other_vector = info['vector']
        points = calculate_intersection_points(current_position, other_position, radius)
        if points:
            intersections.append(points)
    return intersections

def check_border_intersection(current_position, radius, width, height):
    """Check if a circle intersects with the borders of the image."""
    x, y = current_position
    intersections = []
    
    # Check intersection with the left border (x = 0)
    if x - radius < border_buffer:
        intersections.append((0, y))
    
    # Check intersection with the right border (x = width)
    if x + radius > width - border_buffer:
        intersections.append((width, y))
    
    # Check intersection with the top border (y = 0)
    if y - radius < border_buffer:
        intersections.append((x, 0))
    
    # Check intersection with the bottom border (y = height)
    if y + radius > height - border_buffer:
        intersections.append((x, height))
    
    return intersections

def calc_cross_product(v1, v2):
        return v1[0] * v2[1] - v1[1] * v2[0]

def calc_dot_product(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def normalize_vector(vector):
    norm = math.sqrt(vector[0]**2 + vector[1]**2)
    if norm == 0:
        return vector  # Return the original vector if its norm is 0 (to avoid division by zero)
    return (vector[0] / norm, vector[1] / norm)

def avoid_collisions(current_position, normalized_vector, intersections, border_intersections):
     """Adjust movements to avoid collisions."""
     print("I love pedro")
     # Process intersections with other bots
     for point in intersections:
         vector_to_point = (point[0] - current_position[0], point[1] - current_position[1])
         normalized_vector_to_point = normalize_vector(vector_to_point)
       
         dot = calc_dot_product(normalized_vector, normalized_vector_to_point)
        
         if dot < 0:
             print("Collision behind, move forward")
             MoveForward(0.1)
             return
         else:
             print("Collision in front, avoid to the side")
             # Decide to spin left or right based on cross product sign
             cross = calc_cross_product(normalized_vector, normalized_vector_to_point)
             if cross > 0:
                 print("Avoid to the left")
                 SpinLeft(0.3)
             else:
                 print("Avoid to the right")
                 SpinRight(0.3)
             return
   
     # Process intersections with borders
     for border in border_intersections:
         vector_to_border = (border[0] - current_position[0], border[1] - current_position[1])
         normalized_vector_to_border = normalize_vector(vector_to_border)
       
         dot = calc_dot_product(normalized_vector, normalized_vector_to_border)
       
         if dot < 0:
             print("Border behind, move forward")
             MoveForward(0.1)
             return
         else:
             print("Border in front, avoid to the side")
             # Decide to spin left or right based on cross product sign
             cross = calc_cross_product(normalized_vector, normalized_vector_to_border)
             if cross > 0:
                 print("Spin to the left to avoid border")
                 SpinLeft(0.3)
             else:
                 print("Spin to the right to avoid border")
                 SpinRight(0.3)
             return

     print("No collisions detected")

def pathing_light():
    print("I love beer")
    global robot_data
    global client_id
    global last_spin_time
    global notfinished
    global target_position  # New global to hold target position
    global target_tolerance
    global radius
    try:
        current_position = robot_data[client_id]['position']
        current_vector = robot_data[client_id]['vector']
    except KeyError:
        print(f"No key with that name: {client_id}")
        return

    intersections = check_intersections(current_position, current_vector, radius)
    border_intersections = check_border_intersection(current_position, radius, 1280, 720)
    if not notfinished:
        return
    if intersections or border_intersections:
        avoid_collisions(current_position, current_vector, intersections, border_intersections)
    elif target_position is not None:  # If a target position is set, move towards it
        if not move_to_position(current_position, current_vector, target_position, target_tolerance):
            notfinished = False
            Stop()
            print(f"Arrived at target position: {target_position}")
        return
    else:
        if time.ticks_diff(last_spin_time, time.ticks_ms()) < 0:
            ldr_readings, distance_readings, angles = SpinTop(10, 0.1)
            highest_ldr_value = max(ldr_readings)
            highest_ldr_index = ldr_readings.index(highest_ldr_value)
            highest_ldr_angle = angles[highest_ldr_index]
            distance = distance_readings[highest_ldr_index]
            print(distance)
            print(highest_ldr_value)
            if highest_ldr_value > ldr_max_threshold and distance < 170:
                
                print("Found it!")
                notfinished = False
                client.publish(topics['send'], f"foundit {current_position}")
                Stop()   
                print(f"Current position: {current_position}")
            elif highest_ldr_value >= ldr_min_threshold:
                steer_to_angle(highest_ldr_angle)  
            else:
                print(f"LDR values to low ({ldr_min_threshold})")
            last_spin_time = time.ticks_add(time.ticks_ms(), int(300))
            
        else:
            MoveForwardCont()
        
def steer_to_vector(current_vector, target_vector):
    print("I love chicken")
    current_vector = normalize_vector(current_vector)
    target_vector = normalize_vector(target_vector)

    dot_product = calc_dot_product(current_vector, target_vector)
    cross_product = calc_cross_product(current_vector, target_vector)
    
    turn_rate = 0.1
    speed = 0.2
    angle_radians = math.acos(dot_product)
    angle_degrees = math.degrees(angle_radians)
    print(f"dot: {dot_product}")
    print(f"angle: {angle_degrees}")
    print(f"cross: {cross_product}")
    # If the dot product is close to 1, move forward
    if dot_product > 0.9659:
        MoveForward(speed)
    else:
        # Adjust direction
        if cross_product > 0:
            SpinRight(turn_rate)
        else:
            SpinLeft(turn_rate)

def move_to_position(current_position, current_vector, target_position, tolerance):
    print("I love burger")
    target_x, target_y = target_position
    current_x, current_y = current_position
    
    
    distance = math.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)
    
    #distance_x = abs(target_x - current_x)
    #distance_y = abs(target_y - current_y)
    print(f"distance: {distance}")
    if distance <= tolerance:
        print("withing distance")
        return False  
    # distance_x = abs(target_x - current_x)
    # distance_y = abs(target_y - current_y)
    # print(distance_x)
    # print(distance_y)
    # if distance_x <= tolerance and distance_y <= tolerance:
    #     print("withing distance")
    #     return False   
    target_vector = (target_x - current_x, target_y - current_y)
    steer_to_vector(current_vector, target_vector)
    return True  

    
def steer_to_angle(target_angle):
    #target_angle %= 360  # Normalize the target angle to 0-359 degrees
    print(target_angle)
    if 0 <= target_angle <= 90:
        SpinRight(target_angle / 90)
    elif 91 <= target_angle <= 180:
        SpinLeft((target_angle - 90) / 90)
    
# MQTT callbacks
def on_message(topic, msg):
    print("I love cheese")
    global topics
    global robot_data
    global pos_updated
    global target_position
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
        if message.startswith('foundit'):
            try:
                coordinates_str = message.replace("foundit {", "").replace("}", "")
                x_str, y_str = coordinates_str.split(',')
                x = int(x_str.strip())
                y = int(y_str.strip())
                print(f"Received coordinates: x={x}, y={y}")
                target_position = (x, y)  # Set the target position for the robot
                #notfinished = True  # Ensure pathing_light continues to run
            except ValueError:
                print("Error: Unable to parse coordinates from the received message.")
        else:
            try:
                data = json.loads(message)
                # If message is valid JSON, it is assumed to contain position data
                for robot_id, robot_info in data.items():
                    if robot_id != client_id:   
                        if robot_id in robot_data:
                            robot_data[robot_id]['position'] = robot_info['position']
                            robot_data[robot_id]['vector'] = robot_info['vector']
                        else:
                            robot_data[robot_id] = {'position': robot_info['position'], 'vector': robot_info['vector']}
                    # print(f"Updated robot '{robot_id}' position: {robot_data[robot_id]['position']}, angle: {robot_data[robot_id]['angle']}")
                    else:
                    # print(f"Current robot: {client_id} position: {robot_info['position']}, angle: {robot_info['angle']}")
                        robot_data[robot_id] = {'position': robot_info['position'], 'vector': robot_info['vector']}
                
                print("Updated robot data:", robot_data)
                pos_updated = True
            except ValueError:
                # If message is not valid JSON, it is assumed to be a command
                print(f"Received command: {message} on {topic}")
                handle_command(message)
    else:
        print(f"Received message on unknown topic {topic}: {message}")
        handle_command(message)

def handle_command(command):
    global started
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
        SpinLeft(0.3)
    elif command == "SPIN_RIGHT":
        SpinRight(0.3)
    elif command == "SPIN_TOP":
        SpinTop(10.0, 1)
    elif command == "START":
        started = True
    elif command == "STOP":
        Stop()
    elif command == "REBOOT":
        reboot()
    else:
        print("Invalid command.")

# Connect to MQTT broker and start listening for commands
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


#==============================SETUP============================
Stop()
#==============================WIFI=============================
# Activate the Pico LAN
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
print(sta_if.ifconfig()[0])  # Print the IP on the serial
last_update = time.ticks_ms()



connect_mqtt()
# Setup MQTT
distance_sensor.start()
#==============================Main loop========================
while True:
    try:
        client.check_msg()  # Check for new messages
        #print(ldr.read_u16())
        #print(f"distance: {distance_sensor.read()}")
        #print(f". :{ldr.read_u16()}")
        #print("panmotor")
        #PanMotor.duty_u16(5000)
        # Check if it's time to send an update
        
        if time.ticks_diff(time.ticks_ms(), last_update) > update_interval and started and notfinished:
            client.publish(topics['send'], b"request_positions")
            last_update = time.ticks_ms()  # Update the last update time
        
        if pos_updated and notfinished:
            #time_start = time.ticks_ms()
            pathing_light()
            pos_updated = False
            #print(time.ticks_diff(time.ticks_ms(), time_start))
        if not notfinished:
            distance_sensor.stop()
            
    except OSError as e:
        print(f"Error in main loop: {e}")
        time.sleep(2)  # Wait before retrying
        connect_mqtt()
        time.sleep(0.01)
    except Exception as e:
       print(e)
    time.sleep(0.01)