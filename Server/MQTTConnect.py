import time
from paho.mqtt import client as mqtt_client
from Camera.marker_detection import MarkerDetector
# connecting to the server 
broker = 'localhost'
port = 1883
client_id = 'server'
connected_bots = []
real_bots = []
CAMERA_ATTACHED = True
if CAMERA_ATTACHED:
    marker_detection = MarkerDetector(cameraSource=0, camType=0, debug=False)

#define on connect
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Server connected to MQTT Broker!")
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



def setup_bots(client, threshold=10):
    '''
    get list of connected bots. get initial positions. move one bot. get positions. mark bot that moved as the marker id. save to real_bots. 
    '''
    if CAMERA_ATTACHED:
        print("Connected bots")
        print("Connected bots")
        for bot in connected_bots:
            marker_list = marker_detection.detectMarkers()
            client.publish(f"robots/{bot}/receive", f"MOVE_FORWARD")
            time.sleep(2)
            new_marker_list = marker_detection.detectMarkers()

            for i in range(len(marker_list)):
                old_position = marker_list[i]['position']
                new_position = new_marker_list[i]['position']
                delta_x = abs(new_position[0] - old_position[0])
                delta_y = abs(new_position[1] - old_position[1])
                if delta_x > threshold or delta_y > threshold:
                    real_bots.append({
                        'robot_id': bot,
                        'marker_id': marker_list[i]['id'] 
                    })
        

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
