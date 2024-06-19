import time
from paho.mqtt import client as mqtt_client
import json
import threading
from ImageProcessing.vector_marker_detection import MarkerDetector


 # This ensures Camera is imported
# connecting to the server 
broker = 'localhost'
port = 1883
client_id = 'server'
bots_mqtt = []
bots_matched = {}
bots_position = {}

started = False

def start_camera():
    '''
    Setup the video feed needed to get the marker positions.
    Waits 5 seconds to give the streams some time to start up.
    '''
    global detector
    detector  = MarkerDetector(cameraSource1='http://localhost:5005/video_feed', camType1=2, enableCam2=True, cameraSource2=0, camType2=0, debug=False)
    time.sleep(5)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Server connected to MQTT Broker!")
        client.subscribe("server/info")
        client.subscribe("swarm/register")
    else:
        print("Failed to connect, return code", rc)

def on_message(client, userdata, msg):
    print(f"Received message on topic {msg.topic} Message: {msg.payload.decode()}")
    global bots_mqtt
    global started

    if msg.topic == "swarm/register":
        robot_id = msg.payload.decode()
        topic_receive = f"robots/{robot_id}/receive"
        topic_send = f"robots/{robot_id}/send"
        client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        print(f"Assigned ID {robot_id} to new robot. Config sent.")
        bots_mqtt.append(robot_id)
        print(f"Created topics for robot {robot_id}: {topic_receive}, {topic_send}")
        client.subscribe(topic_send)
        print(f"Subscribed to {topic_send}")

    elif msg.topic == "server/info":
        payload = msg.payload.decode()
        if payload == "start":
            print("Received 'start' message on 'server/info' topic")
            started = True
            start_camera()
            setup_bots(client)
            for robots in bots_mqtt:
                positions = get_bot_positions(client)
                print(positions)
                client.publish(f"robots/{robots}/receive", positions)
                client.loop()
                client.publish(f"robots/{robots}/receive", "START")
                client.loop()
        elif payload == "positions":
            get_bot_positions(client)
        else:
            print("Unknown payload on 'server/info' topic")
 
    else:
        for robot_id in bots_mqtt:
            print(robot_id)
            if msg.topic == f"robots/{robot_id}/send":
                payload = msg.payload.decode()
                if payload == "request_positions":
                    positions = get_bot_positions(client)
                    #print(positions)
                    client.publish(f"robots/{robot_id}/receive", positions)
                    print(f"Sent positions to robots/{robot_id}/receive")
                elif payload.startswith("foundit"):
                    try:
                        # Extract coordinates from the payload
                        start_idx = payload.index("[") + 1
                        end_idx = payload.index("]")
                        coordinates_str = payload[start_idx:end_idx]
                        coordinates = list(map(int, coordinates_str.split(",")))

                        if len(coordinates) == 2:
                            x = coordinates[0]
                            y = coordinates[1]

                            for other_robot_id in bots_mqtt:
                                client.publish(f"robots/{other_robot_id}/receive", f"foundit {{{x},{y}}}")
                                print(f"Broadcasted position as foundit {{{x},{y}}} to robots/{other_robot_id}/receive")
                        else:
                            print("Error: Invalid number of coordinates.")
                    
                    except ValueError as ve:
                        print(f"Error parsing coordinates: {ve}")
                    except Exception as e:
                        print(f"Error processing 'foundit' payload: {e}")
                else:
                 print(f"Unknown payload on robots/{robot_id}/send topic")

# todo:
# - potentially add an error loop when it cannot find a match because it misses a detection that particular look
def setup_bots(client, threshold=10):
    '''
    Couples the marker id to a robot id by checking the position, moving the bot and checking which bot moved.
    param client: mqtt client
    param threshold: how much the bots have to move in x or y
    '''
    global bots_mqtt
    global bots_matched
    global detector
    
    # check if there are bots connected
    print(bots_mqtt)
    if len(bots_mqtt) < 1:
        print("no bots connected")
        return
    # get the initial positions to compare against later.
    start_positions = detector.detectMarkers()
    for bot in bots_mqtt:
        # send each bot forward and wait 3 seconds, then get the new positions.
        client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
        client.loop()
        print("Command sent to bot:", bot)
        time.sleep(3)
        new_positions = detector.detectMarkers()
        # check if the detection worked
        if not new_positions:
            print("No markers detected after bot move.")
            continue
        
        # go through each marker and compare them
        for new_marker in new_positions:
            new_x, new_y = new_marker['position']
            for start_marker in start_positions:
                if new_marker['id'] == start_marker['id']:  
                    start_x, start_y = start_marker['position']

                    print(f"Comparing positions for Marker ID {new_marker['id']}")
                    print(f"New position: {new_marker['position']}")
                    print(f"Start position: {start_marker['position']}")

                    x_moved = abs(new_x - start_x)
                    y_moved = abs(new_y - start_y)

                    print(f"Movement detected - x: {x_moved}, y: {y_moved}")

                    # if the marker has moved more than the threshold, add it to the bots_matched dictionary
                    if x_moved > threshold or y_moved > threshold:
                        print(f"Bot {bot} matched with Marker ID {new_marker['id']}")
                        bots_matched.update({bot: new_marker['id']})
                        start_positions = new_positions
                        break  # Exit inner loop once a match is found

    print("Bots matched:", bots_matched)

def get_bot_positions(client):
    
    global detector
    '''
    get bot position and publish them
    '''
    global bots_matched
    global bots_position
    print(started)
    if started == True:
        print("get pos")
        markers = detector.detectMarkers()
        marker_id_to_position = {marker['id']: {'position': marker['position'], 'vector': marker['vector']} for marker in markers}

        for bot, marker_id in bots_matched.items():
            if marker_id in marker_id_to_position:
                bots_position[bot] = marker_id_to_position[marker_id]
                #print("match")

        bots_position_json = json.dumps(bots_position)
        #print("bot positions:", bots_position_json)
        return bots_position_json
    else:
        print("vergeten te starten bruh")

def get_specific_bot_position(bot_id):
    global bots_position
    if bot_id in bots_position:
        specific_position = json.dumps({bot_id: bots_position[bot_id]})
        return specific_position
    else:
        print(f"No position found for robot {bot_id}")
        return None
    
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