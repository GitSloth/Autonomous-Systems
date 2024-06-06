import time
from paho.mqtt import client as mqtt_client

#from ImageProcessing.marker_detection import MarkerDetector
 # This ensures Camera is imported
# connecting to the server 
broker = 'localhost'
port = 1883
client_id = 'server'
connected_bots = []
real_bots = []

CAMERA_ATTACHED = False
#if CAMERA_ATTACHED:
#    detector  = MarkerDetector(cameraSource=0, camType=0, debug=False)
#    print("cam")
#WEBOTSCAM_ATTACHED = True
#if WEBOTSCAM_ATTACHED:
#    webots_detector = MarkerDetector(cameraSource='http://localhost:5000/video_feed', camType=2, debug=False)

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
        connected_bots.append(robot_id)
        #client.publish(f"robots/{robot_id}/config", f"{topic_receive},{topic_send}")
        print(f"Created topics for robot {robot_id}: {topic_receive}, {topic_send}")
    elif msg.topic == "server/info":
        payload = msg.payload.decode()
        if payload == "start":
            print("Received 'start' message on 'server/info' topic")
            #setup_bots(client)
        else:
            print("make the convo brief")


# def setup_bots(client, threshold=10):
#     global connected_bots
#     global real_bots
#     global detector
#     global webots_detector
#     all_marker_info = []
#     print(connected_bots)
#     if len(connected_bots) < 1:
#         print("no bots")
#         return
#     if CAMERA_ATTACHED:
#         print("Detecting markers from the real camera...")
#         markerInfoList = detector.detectMarkers()
#         all_marker_info = markerInfoList
        
#     if WEBOTSCAM_ATTACHED:
#         print("Detecting markers from the Webots camera...")
#         webots_detector = MarkerDetector(cameraSource='http://localhost:5005/video_feed', camType=2, debug=True)
#         all_marker_info.append(webots_detector.detectMarkers())
        
#         #all_marker_info.extend(markerInfoList)
#     print(f"Markers: {len(all_marker_info)}")
#     # Process combined marker info list
#     for bot in connected_bots:
#         new_marker_list = []
#         client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
#         client.loop()  # Force the network loop to process the message immediately
#         time.sleep(5)
#         # start_time = time.time()
#         # while True:
#         #     current_time = time.time()
#         #     if current_time - start_time > 2:
#         #         break

        
        
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
    #     for bot in connected_bots:
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
