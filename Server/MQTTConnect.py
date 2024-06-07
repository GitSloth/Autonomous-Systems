import time
from paho.mqtt import client as mqtt_client

from ImageProcessing.marker_detection_2 import MarkerDetector2


 # This ensures Camera is imported
# connecting to the server 
broker = 'localhost'
port = 1883
client_id = 'server'
bots_mqtt = []
bots_matched = {}
bots_position = {}

def start_camera():
    global detector
    detector = MarkerDetector2(cameraSource1='http://localhost:5005/video_feed', camType1=2, enableCam2=True, cameraSource2=0, camType2=0, debug=True)
    time.sleep(5)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Server connected to MQTT Broker!")
        client.subscribe("server/info")
        client.subscribe("swarm/register")
    else:
        print("Failed to connect, return code", rc)

def on_message(client, userdata, msg):
    print(f"printers")
    global next_robot_id
    if msg.topic == "swarm/register":
        robot_id = msg.payload.decode()
        topic_receive = f"robots/{robot_id}/receive"
        topic_send = f"robots/{robot_id}/send"
        client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        print(f"Assigned ID {robot_id} to new robot. Config sent.")
        bots_mqtt.append(robot_id)
        #client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        print(f"Created topics for robot {robot_id}: {topic_receive}, {topic_send}")
    elif msg.topic == "server/info":
        payload = msg.payload.decode()
        if payload == "start":
            print("Received 'start' message on 'server/info' topic")
            start_camera()
            setup_bots(client)
        else:
            print("make the convo brief")

def setup_bots(client, threshold=10):
    global bots_mqtt
    global bots_matched
    global detector
    print(bots_mqtt)
    if len(bots_mqtt) < 1:
        print("no bots")
        return

    start_positions = detector.detectMarkers()
    print("Initial positions detected:")
    for marker in start_positions:
        print(f"Marker ID: {marker['id']}, Position: {marker['position']}")

    for bot in bots_mqtt:
        client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
        client.loop()
        print("Command sent to bot:", bot)
        time.sleep(3)
        new_positions = detector.detectMarkers()
        # print("start pos")
        # print(start_positions)
        # print("new pos")
        # print(new_positions)

        if not new_positions:
            print("No markers detected after bot move.")
            continue

        for new_marker in new_positions:
            new_x, new_y = new_marker['position']
            for start_marker in start_positions:
                if new_marker['id'] == start_marker['id']:  # Ensure matching marker IDs
                    start_x, start_y = start_marker['position']

                    print(f"Comparing positions for Marker ID {new_marker['id']}")
                    print(f"New position: {new_marker['position']}")
                    print(f"Start position: {start_marker['position']}")

                    x_moved = abs(new_x - start_x)
                    y_moved = abs(new_y - start_y)

                    print(f"Movement detected - x: {x_moved}, y: {y_moved}")

                    if x_moved > threshold or y_moved > threshold:
                        print(f"Bot {bot} matched with Marker ID {new_marker['id']}")
                        bots_matched.update({bot: new_marker['id']})
                        start_positions = new_positions
                        break  # Exit inner loop once a match is found

    print("Bots matched:", bots_matched)
# def setup_bots(client, threshold=10):
#     global bots_mqtt
#     global bots_matched
#     global detector
#     print(bots_mqtt)
#     if len(bots_mqtt) < 1:
#         print("no bots")
#         return
#     start_positions = detector.detectMarkers()
#     print("start pos before for loop")
#     print(start_positions)
#     for bot in bots_mqtt:
#         client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
#         client.loop()
#         print("test1")
#         time.sleep(3)
#         print("test2")
#         new_positions = detector.detectMarkers()
#         print("start pos")
#         print(start_positions)
#         print("new pos")
#         print(new_positions)
#         if not new_positions:
#             print("No markers detected after bot move.")
#             continue
#         for new_marker, start_marker in zip(new_positions, start_positions):
#             new_x, new_y = new_marker['position']
#             start_x, start_y = start_marker['position']
#             #print(new_marker['position'])
#             #print(start_marker['position'])
#             x_moved = abs(new_x - start_x)
#             y_moved = abs(new_y - start_y)
#             print(x_moved)
#             print(y_moved)
#             if x_moved > threshold or y_moved > threshold:
#                 print(f"appending {bot}, {new_marker['id']}")
#                 bots_matched.update({bot:new_marker['id']})
#                 start_positions = new_positions
                
#     print(bots_matched)

    
#         if CAMERA_ATTACHED:
#             new_marker_list = detector.detectMarkers()

#         if WEBOTSCAM_ATTACHED:
#             #new_marker_list.extend(webots_detector.detectMarkers())
#             new_marker_list = webots_detector.detectMarkers()
#         print(new_marker_list)
        
#         for i in range(len(all_marker_info)):
#             old_position = markerInfoList[i]['position']
#             new_position = new_marker_list[i]['position']
#             delta_x = abs(new_position[0] - old_position[0])
#             delta_y = abs(new_position[1] - old_position[1])
#             if delta_x > threshold or delta_y > threshold:
#                 real_bots.append({
#                     'robot_id': bot,
#                     'marker_id': new_marker_list[i]['id']
#                 })
#         all_marker_info = new_marker_list

#     if real_bots is not None:
#             for combo in real_bots:
#                 print(f"Robot_id: {combo['robot_id']}, marker_id: {combo['marker_id']}")
    # print("Combined marker information:", real_bots)
    # if CAMERA_ATTACHED:
    #     print("Connected bots")
    #     for bot in bots_mqtt:
    #         while True:  # Continuously detect markers and move the robot
    #             markerInfoList = detector.detectMarkers()
    #             client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
    #             time.sleep(2)
    #             new_marker_list = detector.detectMarkers()

    #             for i in range(len(markerInfoList)):
    #                 old_position = markerInfoList[i]['position']
    #                 new_position = new_marker_list[i]['position']
    #                 delta_x = abs(new_position[0] - old_position[0])
    #                 delta_y = abs(new_position[1] - old_position[1])
    #                 if delta_x > threshold or delta_y > threshold:
    #                     real_bots.append({
    #                         'robot_id': bot,
    #                         'marker_id': markerInfoList[i]['id'] 
    #                     })
    #             break  # Exit the loop after one iteration to move to the next robot or next execution of setup_bots
    # else:
    #     print("Camera not attached")


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
