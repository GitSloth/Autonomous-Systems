import time
from paho.mqtt import client as mqtt_client
import json
import threading
from ImageProcessing.vector_marker_detection import MarkerDetector
import logging

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

def start_camera():
    '''
    Setup the video feed needed to get the marker positions.
    Waits 5 seconds to give the streams some time to start up.
    '''
    global detector
    detector  = MarkerDetector(cameraSource1=0, camType1=0, enableCam2=True, cameraSource2='http://localhost:5005/video_feed', camType2=2, debug=True)
    time.sleep(5)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logger.info("Server connected to MQTT Broker!")
        client.subscribe("server/info")
        client.subscribe("swarm/register")
    else:
        print("Failed to connect, return code", rc)

def on_message(client, userdata, msg):
    logger.info(f"Received message on topic {msg.topic} Message: {msg.payload.decode()}")
    global bots_mqtt
    global started

    if msg.topic == "swarm/register":
        robot_id = msg.payload.decode()
        with lock:
            topic_receive = f"robots/{robot_id}/receive"
            topic_send = f"robots/{robot_id}/send"
            client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
            logger.info(f"Assigned ID {robot_id} to new robot. Config sent.")
            bots_mqtt.append(robot_id)
            logger.info(f"Created topics for robot {robot_id}: {topic_receive}, {topic_send}")
            client.subscribe(topic_send)
            logger.info(f"Subscribed to {topic_send}")

    elif msg.topic == "server/info":
        payload = msg.payload.decode()
        if payload == "start":
            logger.info("Received 'start' message on 'server/info' topic")
            started = True
            start_camera()
            setup_bots(client)
            for robots in bots_mqtt:
                positions = get_bot_positions(client)
                print(positions)
                client.publish(f"robots/{robots}/receive", "START")
                client.loop()
                client.publish(f"robots/{robots}/receive", positions)
                client.loop()
        elif payload == "positions":
            get_bot_positions(client)
        else:
            logger.warning("Unknown payload on 'server/info' topic")
 
    else:
        #with lock:
        for robot_id in bots_mqtt:
            if msg.topic == f"robots/{robot_id}/send":
                payload = msg.payload.decode()
                if payload == "request_positions":
                    positions = get_bot_positions(client)
                    client.publish(f"robots/{robot_id}/receive", positions)
                    logger.info(f"Sent positions to robots/{robot_id}/receive")
                elif payload.startswith("foundit"):
                    handle_foundit_payload(client,payload)
                else:
                    logger.warning(f"Unknown payload on robots/{robot_id}/send topic")

def handle_foundit_payload(client, payload):
    try:
        # Extract coordinates from the payload
        start_idx = payload.index("[") + 1
        end_idx = payload.index("]")
        coordinates_str = payload[start_idx:end_idx]
        coordinates = list(map(int, coordinates_str.split(",")))

        if len(coordinates) == 2:
            x = coordinates[0]
            y = coordinates[1]
            with lock:
                for other_robot_id in bots_mqtt:
                    client.publish(f"robots/{other_robot_id}/receive", f"foundit {{{x},{y}}}")
                    logger.info(f"Broadcasted position as foundit {{{x},{y}}} to robots/{other_robot_id}/receive")
        else:
            logger.error("Error: Invalid number of coordinates.")
    
    except ValueError as ve:
        logger.error(f"Error parsing coordinates: {ve}")
    except Exception as e:
        logger.error(f"Error processing 'foundit' payload: {e}")
# todo:
# - potentially add an error loop when it cannot find a match because it misses a detection that particular look
def setup_bots(client, threshold=10):
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
    start_positions = detector.detectMarkers()
    for bot in bots_mqtt:
        # send each bot forward and wait 3 seconds, then get the new positions.
        client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
        client.loop()
        logger.info(f"Command sent to bot: {bot}")
        time.sleep(3)
        new_positions = detector.detectMarkers()
        # check if the detection worked
        if not new_positions:
            logger.warning("No markers detected after bot move.")
            continue
        
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
                    if x_moved > threshold or y_moved > threshold:
                        bots_matched.update({bot: new_marker['id']})
                        start_positions = new_positions
                        
                        logger.info(f"Bot {bot} matched with Marker ID {new_marker['id']}")
                        break  # Exit inner loop once a match is found

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
