import time
from paho.mqtt import client as mqtt_client
import json
import threading
from ImageProcessing.vector_marker_detection import MarkerDetector
import logging
import heapq
import math
import numpy as np
from scipy.optimize import linear_sum_assignment

#camera setup parameters
cameraSource1 = 0
camType1 = 0
enableCam2 = True
cameraSource2 ='http://localhost:5005/video_feed' 
camType2 = 2
debug = True

#movement data
movement_threshold = 10


# Logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# connecting to the server 
broker = 'localhost'
port = 1883
client_id = 'server'
bots_mqtt = []
bots_matched = {}
bots_position = {}

started = False

lock = threading.Lock()

#Initializes camera and marker detection
def start_camera():
    '''
    Setup the video feed needed to get the marker positions.
    Waits 5 seconds to give the streams some time to start up.
    '''
    global detector
    retry_attempts = 3
    attempt = 0
    camera_success = False

    while attempt < retry_attempts and not camera_success:
        try:
            detector = MarkerDetector(cameraSource1, camType1, enableCam2, cameraSource2, camType2, debug)
            time.sleep(5)

            if detector_check(detector):
                success = True
                logger.info("Camera Start Successful")
            else:
                raise RuntimeError("MarkerDetector Initialization Failed")
        
        except Exception as e:
            attempt += 1
            logger.error(f"Error With Initialization Camera: {e} Attempt {attempt} of {retry_attempts}")
            time.sleep(2) #wait a little before next attempt
    
    if not success:
        logger.critical("Failed To Start Camera After Multiple Attempts")

#Check if detector is properly initialized
def detector_check(detector):
    try:
        detector.detectMarkers() #tries to call function to see if properly initialized
        return True
    except Exception as e:
        logger.error(f"Problem with Detector Initialization: {e}")
        return False


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Server connected to MQTT Broker!")
        client.subscribe("server/info")
        client.subscribe("swarm/register")
    else:
        print("Failed to connect, return code", rc)

# Main on_message callback for handling incoming MQTT messages
def on_message(client, userdata, msg):
    logger.info(f"Received message on topic {msg.topic} Message: {msg.payload.decode()}")
    
    global bots_mqtt
    global started
    
    # Route the message to the appropriate handler based on the topic
    if msg.topic == "swarm/register":
        handle_swarm_register(client, msg)
    elif msg.topic == "server/info":
        handle_server_info(client, msg)
    else:
        handle_robot_messages(client, msg)

# Handle registration of a new robot
def handle_swarm_register(client, msg):
    robot_id = msg.payload.decode()
    with lock:
        # Define topics for receiving and sending messages for the new robot
        topic_receive = f"robots/{robot_id}/receive"
        topic_send = f"robots/{robot_id}/send"
        
        # Publish configuration to the new robot
        client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        logger.info(f"Assigned ID {robot_id} to new robot. Config sent.")
        
        # Add the robot to the list of bots and subscribe to its send topic
        bots_mqtt.append(robot_id)
        logger.info(f"Created topics for robot {robot_id}: {topic_receive}, {topic_send}")
        client.subscribe(topic_send)
        logger.info(f"Subscribed to {topic_send}")

# Handle server commands and information
def handle_server_info(client, msg):
    payload = msg.payload.decode()
    if payload == "start":
        handle_start_message(client)
    elif payload == "positions":
        get_bot_positions(client)
    else:
        logger.warning("Unknown payload on 'server/info' topic")

# Handle the 'start' command from the server
def handle_start_message(client):
    logger.info("Received 'start' message on 'server/info' topic")
    global started
    started = True
    
    # Start the camera and setup bots
    start_camera()
    setup_bots(client)
    
    # Send 'START' message and positions to each robot
    for robot_id in bots_mqtt:
        positions = get_bot_positions(client)
        logger.info(f"Sending START and positions to robots/{robot_id}/receive")
        send_start_and_positions(client, robot_id, positions)

# Helper function to send 'START' command and positions to a robot
def send_start_and_positions(client, robot_id, positions):
    client.publish(f"robots/{robot_id}/receive", "START")
    client.loop()
    client.publish(f"robots/{robot_id}/receive", positions)
    client.loop()

# Handle messages from individual robots
def handle_robot_messages(client, msg):
    for robot_id in bots_mqtt:
        if msg.topic == f"robots/{robot_id}/send":
            payload = msg.payload.decode()
            if payload == "request_positions":
                # Respond to position requests from robots
                positions = get_bot_positions(client)
                client.publish(f"robots/{robot_id}/receive", positions)
                logger.info(f"Sent positions to robots/{robot_id}/receive")
            elif payload.startswith("foundit"):
                # Handle 'foundit' payloads from robots
                handle_foundit_payload(client, payload)
            else:
                logger.warning(f"Unknown payload on robots/{robot_id}/send topic")

def handle_foundit_payload(client, payload):
    global bots_mqtt
    try:
        # Extract target coordinates from the payload
        start_idx = payload.index("[") + 1
        end_idx = payload.index("]")
        coordinates_str = payload[start_idx:end_idx]
        coordinates = list(map(int, coordinates_str.split(",")))

        if len(coordinates) != 2:
            logger.error("Invalid number of coordinates received.")
            return
        
        x_center, y_center = coordinates[0], coordinates[1]

        # Create target circle around the received center coordinates
        robot_target_positions = create_target_circle( x_center, y_center)

        # Use Dijkstra's algorithm to distribute targets efficiently
        distribute_targets(client, robot_target_positions)

    except ValueError as ve:
        logger.error(f"Error parsing coordinates: {ve}")
    except Exception as e:
        logger.error(f"Error processing 'foundit' payload: {e}")

#Calculates the euclidian distance between two points
def euclidean_distance(vector1, vector2):
    return np.linalg.norm(np.array(vector1) - np.array(vector2))

#creates a circle around the target and creates coordinates to act as parking spaces once the target has been found
def create_target_circle( x_coordinate, y_coordinate):
    global bots_mqtt

    if not bots_mqtt:
        logger.warning("No robots connected.")
        return
    
    num_robots = len(bots_mqtt)
    bot_radius = 80
    circle_radius = bot_radius * (num_robots - 1)

    angle_increment = 2 * math.pi / num_robots

    robot_target_positions = []

    for i in range(num_robots):
        angle = i * angle_increment
        x = x_coordinate + circle_radius * math.cos(angle)
        y = y_coordinate + circle_radius * math.sin(angle)
        robot_target_positions.append((x,y))
    return robot_target_positions

#The 'parking spaces' get distributed to the bots based on Dijkstra's algorithm
def distribute_targets(client, robot_target_positions):
    global bots_position, bots_mqtt, logger

    if not bots_mqtt:
        logger.warning("No robots connected.")
        return

    # Prepare data for the Hungarian algorithm
    robot_positions = []
    for bot_id in bots_mqtt:
        if bot_id in bots_position:
            robot_positions.append(bots_position[bot_id]['position'])
        else:
            logger.warning(f"No position data found for robot {bot_id}. Skipping.")
    
    num_robots = len(robot_positions)
    num_targets = len(robot_target_positions)

    if num_robots == 0:
        logger.warning("No robots with position data.")
        return

    if num_targets == 0:
        logger.warning("No target positions provided.")
        return

    # Calculate cost matrix based on Euclidean distances
    cost_matrix = np.zeros((num_robots, num_targets))
    for i in range(num_robots):
        for j in range(num_targets):
            cost_matrix[i, j] = euclidean_distance(robot_positions[i], robot_target_positions[j])

    # Solve the assignment problem using scipy's linear_sum_assignment (Hungarian algorithm)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    # Map robots to their assigned target positions
    assigned_positions = {}
    for i in range(len(row_ind)):
        robot_id = bots_mqtt[row_ind[i]]
        target_position = robot_target_positions[col_ind[i]]
        assigned_positions[robot_id] = target_position

    # Publish new positions to robots based on optimal assignment
    for robot_id, target_position in assigned_positions.items():
        client.publish(f"robots/{robot_id}/receive", f"foundit {{{target_position[0]}, {target_position[1]}}}")
        logger.info(f"Published new position {target_position} to robot {robot_id}")


def setup_bots(client):
    '''
    Couples the marker id to a robot id by checking the position, moving the bot and checking which bot moved.
    param client: mqtt client
    param threshold: how much the bots have to move in x or y
    '''
    global bots_mqtt, bots_matched, detector
    #check if there are any bost connected
    
    if not bots_mqtt:
        logger.info("no bots connected")
        return
    # get the initial positions to compare against later.

    try:
        start_positions = detector.detectMarkers()
        if not start_positions:
            raise RuntimeError("Failed To Detect Initial Markers")
        
        for bot in bots_mqtt:
            try:
                # send each bot forward and wait 3 seconds, then get the new positions.
                client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
                client.loop()
                logger.info(f"Command sent to bot: {bot}")
                time.sleep(3)
                
                new_positions = detector.detectMarkers()

            # check if the detection worked
                if not new_positions:
                    raise RuntimeError("No markers detected after bot move.")
            
                matched = False
            
                # go through each marker and compare them
                for new_marker in new_positions:
                    new_x, new_y = new_marker['position']
                    for start_marker in start_positions:
                        if new_marker['id'] == start_marker['id']:  
                            start_x, start_y = start_marker['position']
                            x_moved = abs(new_x - start_x)
                            y_moved = abs(new_y - start_y)

                            #print(f"Movement detected - x: {x_moved}, y: {y_moved}")

                            # if the marker has moved more than the threshold, add it to the bots_matched dictionary
                            if x_moved > movement_threshold or y_moved > movement_threshold:
                                bots_matched.update({bot: new_marker['id']})
                                start_positions = new_positions
                                logger.info(f"Bot {bot} matched with Marker ID {new_marker['id']}")
                                matched = True
                                break  # Exit inner loop once a match is found
                    
                    if matched:
                        break
                
                if not matched:
                    logger.warning(f"Bot {bot} Could Not Be Matched With Any Marker")
            
            except Exception as e:
                logger.error(f"Error Matching Bot {bot}: {e}")

    except Exception as e:
        logger.critical(f"Failed To Setup Bots {bot}: {e}")    

    logger.info("Bots matched: %s", bots_matched)

def get_bot_positions(client):
    '''
    get bot position and publish them
    '''
    global detector, bots_matched, bots_position
    if not started:
        logger.warning("System not started")
        return {}
    
    markers = detector.detectMarkers()
    marker_id_to_position = {marker['id']: {'position': marker['position'], 'vector': marker['vector']} for marker in markers}
    
    with lock:
        for bot, marker_id in bots_matched.items():
            if marker_id in marker_id_to_position:
                bots_position[bot] = marker_id_to_position[marker_id]

    bots_position_json = json.dumps(bots_position)
    return bots_position_json
    
def periodic_position_updates(client):
    while True:
        get_bot_positions(client)
        time.sleep(1)

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
