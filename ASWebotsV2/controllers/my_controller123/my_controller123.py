import random
import paho.mqtt.client as mqtt
from controller import Robot
import threading
import json
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Initialize motors and sensors
motor1 = robot.getDevice('motor1')
motor2 = robot.getDevice('motor2')
motor3 = robot.getDevice('motor3')
range_finder = robot.getDevice('range-finder')
ldr = robot.getDevice('light sensor')

# Set initial motor velocities
motor1.setVelocity(0)
motor2.setVelocity(0)
motor3.setVelocity(0)

# Enable sensors
ldr.enable(timestep)
range_finder.enable(timestep)

motor1.setPosition(float('inf'))
motor2.setPosition(float('inf'))
motor3.setPosition(float('inf'))

# Motor velocities
forward_velocity = -1.0
backward_velocity = 1.0

# MQTT settings
broker = 'localhost'
port = 1883
client_id = f'robot_{random.randint(0, 10000)}'
topic_register = "swarm/register"
client = None
topics = {}

 
epsilon = 100   
sigma = 50   
cutoff_distance = 5 * sigma  

 
position = [0.0, 0.0]   
base_speed = 0.05   
orientation = 0.0   

 
min_speed = 0.001
max_speed = 0.1

# Movement functions
def MoveForward(duration):
    print("MoveForward called with duration:", duration)
    motor1.setVelocity(forward_velocity)
    motor2.setVelocity(forward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def MoveBack(duration):
    print("MoveBack called with duration:", duration)
    motor1.setVelocity(backward_velocity)
    motor2.setVelocity(backward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinLeft(duration):
    print("SpinLeft called with duration:", duration)
    motor2.setVelocity(backward_velocity)
    motor1.setVelocity(forward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinRight(duration):
    print("SpinRight called with duration:", duration)
    motor1.setVelocity(backward_velocity)
    motor2.setVelocity(forward_velocity)
    end_time = robot.getTime() + duration
    while robot.getTime() < end_time:
        if robot.step(timestep) == -1:
            break
    motor1.setVelocity(0)
    motor2.setVelocity(0)

def SpinTop(speed, duration):
    ldr_readings = []
    distance_readings = []
    step_count = 10
    angle_step = 180 / (step_count - 1)
    initial_position = -90
    for step in range(step_count):
        position_degrees = initial_position + step * angle_step
        position_radians = position_degrees * (3.14159 / 180)
        motor3.setPosition(position_radians)
        motor3.setVelocity(speed)
        end_time = robot.getTime() + duration
        while robot.getTime() < end_time:
            if robot.step(timestep) == -1:
                break
        ldr_value = ldr.getValue()
        ldr_readings.append(ldr_value)
        distance_image = range_finder.getRangeImage()
        distance_value = process_range_image(distance_image)
        distance_readings.append(distance_value)
        
    motor3.setPosition(0)
    print("LDR Readings:", ldr_readings)
    print("Distance Readings:", distance_readings)

def process_range_image(image):
    if len(image) == 0:
        return float('inf')
    min_distance = float('inf')
    for distance in image:
        if distance < min_distance:
            min_distance = distance
    return min_distance
    
    
def calculate_force(position_a, position_b):
    """Calculate the force vector between two bots based on Lennard-Jones potential."""
    dx = position_b[0] - position_a[0]
    dy = position_b[1] - position_a[1]
    r = math.sqrt(dx ** 2 + dy ** 2)
    """No force returns after cutoff distance"""
    if r > cutoff_distance:
        return 0.0, 0.0
    
    if r != 0:
        force_magnitude = 24 * epsilon * (2 * (sigma ** 12 / r ** 13) - (sigma ** 6 / r ** 7))
        force_x = force_magnitude * (dx / r)
        force_y = force_magnitude * (dy / r)
    else:
        force_x, force_y = 0, 0
    
    print(f"Force from ({position_a}) to ({position_b}) -> Force: ({force_x}, {force_y})")  # Debugging force values
    return force_x, force_y
    

def update_position_and_orientation(position, orientation, forces):
    total_force_x, total_force_y = 0, 0
    for fx, fy in forces:
        total_force_x += fx
        total_force_y += fy
    
    resultant_angle = math.atan2(total_force_y, total_force_x) * 180 / math.pi
    resultant_magnitude = math.sqrt(total_force_x ** 2 + total_force_y ** 2)
    
 
    if resultant_angle > 180:
        resultant_angle -= 360
    elif resultant_angle < -180:
        resultant_angle += 360
    
 
    orientation_difference = resultant_angle - orientation
    if orientation_difference > 180:
        orientation_difference -= 360
    elif orientation_difference < -180:
        orientation_difference += 360
    orientation += orientation_difference * 0.1   
    
    speed = min(max_speed, max(min_speed, base_speed * resultant_magnitude))
    
 
    if position[0] < 0 or position[0] > 1000 or position[1] < 0 or position[1] > 500:
 
        SpinRight(2)   
    else:
 
        MoveForward(1)  

    position[0] += speed * math.cos(math.radians(orientation))
    position[1] += speed * math.sin(math.radians(orientation))
    
    print(f"Updated position: {position}, orientation: {orientation}, speed: {speed}")
    return position, orientation, speed

def on_message(client, userdata, msg):
    global topics, position, orientation
    topic = msg.topic
    message = msg.payload.decode()
    
    if topic == f"robots/{client_id}/config":
        config = message.split(',')
        if len(config) == 2:
            topics['receive'] = config[0]
            topics['send'] = config[1]
            client.subscribe(topics['receive'])
            client.publish(topics['send'], f"{client_id} connected successfully")
        else:
            print("Invalid configuration format received.")
    elif topic == "robots/positions":
        try:
            data = json.loads(message)
            forces = []
            for robot_id, robot_info in data.items():
                if robot_id == client_id:
                    position = robot_info['position']
                    orientation = robot_info['angle']
                else:
                    x, y = robot_info['position']
                    other_position = (x, y)
                    fx, fy = calculate_force(position, other_position)
                    forces.append((fx, fy))
                    
            position, orientation, speed = update_position_and_orientation(position, orientation, forces)
            
            perform_movement_based_on_orientation(orientation, speed)
            
        except json.JSONDecodeError:
            print("Invalid JSON format received for robot positions.")
    else:
        handle_command(message)

def handle_command(command):
    print("Received command:", command)
    command = command.strip().upper()
    if command == "MOVE_FORWARD":
        MoveForward(2)
    elif command == "MOVE_BACK":
        MoveBack(1)
    elif command == "SPIN_LEFT":
        SpinLeft(1)
    elif command == "SPIN_RIGHT":
        SpinRight(1)
    elif command == "SPIN_TOP":
        SpinTop(10.0, 1)
    else:
        print("Invalid command.")

def perform_movement_based_on_orientation(orientation, speed):
    print(f"Performing movement based on orientation: {orientation}, speed: {speed}")
    if -45 <= orientation <= 45:
        MoveForward(1)
    elif 135 <= orientation <= 225:
        MoveBack(1)
    elif 45 < orientation < 135:
        SpinLeft(1)
    elif 225 < orientation < 315:
        SpinRight(1)
    else:
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        random_move()

def random_move():
    move_type = random.choice(['MOVE_FORWARD', 'MOVE_BACK', 'SPIN_LEFT', 'SPIN_RIGHT'])
    print(f"Random move chosen:")
    #if move_type == 'MOVE_BACK':
         #MoveForward(1)
    if move_type == 'MOVE_FORWARD':
        MoveBack(1)
    elif move_type == 'SPIN_RIGHT':
        SpinLeft(1)
    elif move_type == 'SPIN_LEFT':
        SpinRight(1)

client = mqtt.Client(client_id)
client.on_message = on_message
def connect_mqtt():
    try:
        client.connect(broker, port)
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/config")
        client.subscribe(f"robots/positions")
        print(f"Subscribed to robots/{client_id}/config")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    except Exception as e:
        print(f"Failed to connect to MQTT Broker: {e}")

def run_mqtt():
    print("Starting")
    connect_mqtt()
    message_thread = threading.Thread(target=lambda: client.loop_forever())
    message_thread.start()
    while robot.step(timestep) != -1:
        pass

run_mqtt()