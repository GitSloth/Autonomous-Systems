import random
import time
from machine import Pin, PWM, ADC, I2C
from vl53l0x import VL53L0X
from umqtt.simple import MQTTClient
import network
import math
import json

# Pins
BUILT_IN_LED = 25  # Built-in LED
FLED = 20  # Front LED Red
BLED = 21  # Back LED Green
PWM_LM = 6  # Left Continuous Servo
PWM_RM = 7  # Right Continuous Servo
PWM_SC = 10  # Panning Servo
LDR_PIN = 27 # ldr sensor
sda_pin = Pin(0) # SDA pin
scl_pin = Pin(1) # SCL pin

# Sensors
ldr = ADC(Pin(LDR_PIN))
ldr_readings = []
distance_readings = []
<<<<<<< Updated upstream
radius = 120
robot_data = {}
=======
>>>>>>> Stashed changes


# Initialize the I2C bus
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
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
# Set up servos
LeftMotor = PWM(Pin(PWM_LM))
LeftMotor.freq(50)
RightMotor = PWM(Pin(PWM_RM))
RightMotor.freq(50)
PanMotor = PWM(Pin(PWM_SC))
PanMotor.freq(50)
stop_duty = 4915
duty_range = 1638

<<<<<<< Updated upstream
left_velocity = 20
=======
# Speed
left_velocity = 25
>>>>>>> Stashed changes
right_velocity = 30

# mqtt settings
broker = '10.198.64.131' # change ip based on network 
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
client = MQTTClient(client_id, broker, port)
topic_register = "swarm/register"
<<<<<<< Updated upstream
topics = {}  # Ensure topics is an empty dictionary initially
=======
topics = {}
>>>>>>> Stashed changes
counter = 0
subscribed_topics = set()
update_interval = 50  # Update interval in milliseconds

# positioning
radius = 80
border_buffer = 40
robot_data = {}
pos_updated = False
started = False
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
    
def set_pan_servo_angle(angle):
    # Convert the angle to a duty cycle
    # 0 degrees corresponds to 1 ms pulse width (duty of 26 out of 1023)
    # 180 degrees corresponds to 2 ms pulse width (duty of 128 out of 1023)
    # Map the angle to a value between 26 and 128
    duty = int((angle / 180.0 * 102) + 26)
    PanMotor.duty(duty)
    
# forward  
def MoveForward(duration):
    global left_velocity
    global right_velocity
    print("MoveForward called with duration:", duration)
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)
<<<<<<< Updated upstream
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
=======
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
>>>>>>> Stashed changes
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def MoveBack(duration):
    print("MoveBack called with duration:", duration)
    set_servo_speed_left(-left_velocity)
    set_servo_speed_right(-right_velocity)
<<<<<<< Updated upstream
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
=======
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
>>>>>>> Stashed changes
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinLeft(duration):
    set_servo_speed_left(0)
    set_servo_speed_right(right_velocity)
<<<<<<< Updated upstream
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
=======
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
>>>>>>> Stashed changes
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinRight(duration):
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(0)
<<<<<<< Updated upstream
    end_time = time.time_ns() + (duration * 1000000000)
    while True:
        if time.time_ns() > end_time:
            break
=======
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
>>>>>>> Stashed changes
    set_servo_speed_left(0)
    set_servo_speed_right(0)

def SpinTop(speed, duration):
    distance_sensor.start()
    ldr_readings = []
    distance_readings = []
    angles = []
    
    duty_cycles = [2000, 3000, 4000, 5000, 6000, 7000, 8000]
    for duty in duty_cycles:
        PanMotor.duty_u16(duty)
        end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
        while time.ticks_diff(end_time, time.ticks_ms()) > 0:
            pass
        ldr_readings.append(ldr.read_u16())
        distance_readings.append(distance_sensor.read())

    PanMotor.duty_u16(5000)
    distance_sensor.stop()
    return ldr_readings, distance_readings, angles

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


def normalize_vector(vector):
    norm = math.sqrt(vector[0]**2 + vector[1]**2)
    if norm == 0:
        return vector  # Return the original vector if its norm is 0 (to avoid division by zero)
    return (vector[0] / norm, vector[1] / norm)

def avoid_collisions(current_position, normalized_vector, intersections, border_intersections):
     """Adjust movements to avoid collisions."""
     def cross_product(v1, v2):
         return v1[0] * v2[1] - v1[1] * v2[0]
   
     def dot_product(v1, v2):
         return v1[0] * v2[0] + v1[1] * v2[1]
     # Process intersections with other bots
     for point in intersections:
         vector_to_point = (point[0] - current_position[0], point[1] - current_position[1])
         normalized_vector_to_point = normalize_vector(vector_to_point)
       
         dot = dot_product(normalized_vector, normalized_vector_to_point)
        
         if dot < 0:
             print("Collision behind, move forward")
             MoveForward(0.1)
             return
         else:
             print("Collision in front, avoid to the side")
             # Decide to spin left or right based on cross product sign
             cross = cross_product(normalized_vector, normalized_vector_to_point)
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
       
         dot = dot_product(normalized_vector, normalized_vector_to_border)
       
         if dot < 0:
             print("Border behind, move forward")
             MoveForward(0.1)
             return
         else:
             print("Border in front, avoid to the side")
             # Decide to spin left or right based on cross product sign
             cross = cross_product(normalized_vector, normalized_vector_to_border)
             if cross > 0:
                 print("Spin to the left to avoid border")
                 SpinLeft(0.3)
             else:
                 print("Spin to the right to avoid border")
                 SpinRight(0.3)
             return

     print("No collisions detected")

def pathing_light():
    global robot_data
    global client_id
    global last_spin_time
    
    try:
        current_position = robot_data[client_id]['position']
        current_vector = robot_data[client_id]['vector']
    except KeyError:
        print(f"No key with that name: {client_id}")
        return

    intersections = check_intersections(current_position, current_vector, radius)
    border_intersections = check_border_intersection(current_position, radius, 1280, 720)
    
    if intersections or border_intersections:
        avoid_collisions(current_position, current_vector, intersections, border_intersections)
    else:
        current_time = robot.getTime()
        if current_time - last_spin_time >= 5:
            ldr_readings, distance_readings, angles = SpinTop(10, 0.1)
            highest_ldr_value = max(ldr_readings)
            highest_ldr_index = ldr_readings.index(highest_ldr_value)
            highest_ldr_angle = angles[highest_ldr_index]
            print(highest_ldr_value)
            if highest_ldr_value > ldr_max_threshold:
                print("Found it!")
                Stop()   
            elif highest_ldr_value >= ldr_min_threshold:
                steer_to_angle(highest_ldr_angle)  
                last_spin_time = current_time
            else:
                print(f"LDR values to low ({ldr_min_threshold})")
                MoveForward(1)
        else:
        MoveForwardCont()
            
def steer_to_angle(target_angle):
    if target_angle < 0:
        target_angle += 180
    elif target_angle > 180:
        target_angle -= 180
    

    if target_angle > 90:
        SpinRight((180 - target_angle) / 90)  
    else:
        SpinLeft(target_angle / 90) 
    
    MoveForward(1)
    
# MQTT callbacks
def on_message(topic, msg):
    print("I love cheese")
    global topics
    global robot_data
<<<<<<< Updated upstream
=======
    global pos_updated
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
            pathing_light()
            client.publish(topics['send'], b"request_positions")
=======
            pos_updated = True
>>>>>>> Stashed changes
        except ValueError:
            # If message is not valid JSON, it is assumed to be a command
            print(f"Received command: {message} on {topic}")
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
<<<<<<< Updated upstream
=======
    elif command == "START":
        started = True
>>>>>>> Stashed changes
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
ssid = 'maters wtnf3'
password = 'troepopruimen'

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

#==============================Main loop========================
while True:
    try:
        client.check_msg()  # Check for new messages
        
        # Check if it's time to send an update
        if time.ticks_diff(time.ticks_ms(), last_update) > update_interval and started:
            client.publish(topics['send'], b"request_positions")
            last_update = time.ticks_ms()  # Update the last update time
        
        if pos_updated:
            time_start = time.ticks_ms()
            pathing_light()
            pos_updated = False
            print(time.ticks_diff(time.ticks_ms(), time_start))
    except OSError as e:
        print(f"Error in MQTT loop: {e}")
        time.sleep(2)  # Wait before retrying
        connect_mqtt()
        time.sleep(0.01)
    except Exception as e:
       print(e)
    time.sleep(0.01)

<<<<<<< Updated upstream
# Listen for MQTT commands
run_mqtt()
=======




>>>>>>> Stashed changes
