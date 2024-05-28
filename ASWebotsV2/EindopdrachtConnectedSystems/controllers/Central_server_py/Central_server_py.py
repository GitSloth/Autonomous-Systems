import json
import logging
import random
import time
from paho.mqtt import client as mqtt_client
import heapq
from typing import List, Tuple

from controller import Supervisor
from controller import Robot
from controller import Camera

from queue import Queue


# Create a Webots robot instance
supervisor = Supervisor() 
obj = supervisor.getFromDef("camera")
camera = Camera("camera")

camera.enable(1000)

width = camera.getWidth()
height = camera.getHeight()



web_robot_0 = supervisor.getFromDef("pick_up_robot_0")
web_robot_1 = supervisor.getFromDef("pick_up_robot_1")
web_robot_2 = supervisor.getFromDef("pick_up_robot_2")
web_robot_3 = supervisor.getFromDef("pick_up_robot_3")

web_robots = [web_robot_0,web_robot_1,web_robot_2,web_robot_3]

Old_locations = [[-1,-1,-1],[-1,-1,-1],[-1,-1,-1],[-1,-1,-1]]

Old_directions = ["off","off","off","off"]
direction_options = ["off","forward","backwards","left","right"]

Old_sensor_values = [[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1]]

broker = '192.168.178.244'
port = 1883
topic_server = "publish/server"
topic_robots = "publish/robots"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'emqx'
password = 'public'

q_w0 = Queue()
q_w1 = Queue()
q_w2 = Queue()
q_w3 = Queue()

queues = [q_w0,q_w1,q_w2,q_w3]

id = 0

client = None

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

FLAG_EXIT = False

grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]


INF = float('inf')

def dijkstra(grid: List[List[int]], source: Tuple[int, int], destination: Tuple[int, int]) -> List[Tuple[int, int]]:
    rows = len(grid)
    cols = len(grid[0])

    distance = [[INF] * cols for _ in range(rows)]
    visited = [[False] * cols for _ in range(rows)]
    parent = [[(-1, -1)] * cols for _ in range(rows)]

    dr = [0, 0, 1, -1]
    dc = [1, -1, 0, 0]

    def is_valid(row: int, col: int) -> bool:
        return 0 <= row < rows and 0 <= col < cols

    def get_weight(row: int, col: int) -> int:
        return grid[row][col]

    pq = []
    heapq.heappush(pq, (0, source))
    distance[source[0]][source[1]] = 0

    while pq:
        dist, current = heapq.heappop(pq)
        curr_row, curr_col = current

        if visited[curr_row][curr_col]:
            continue

        visited[curr_row][curr_col] = True

        if current == destination:
            break

        for i in range(4):
            new_row = curr_row + dr[i]
            new_col = curr_col + dc[i]

            if is_valid(new_row, new_col):
                weight = get_weight(new_row, new_col)
                new_dist = distance[curr_row][curr_col] + weight

                if new_dist < distance[new_row][new_col]:
                    distance[new_row][new_col] = new_dist
                    heapq.heappush(pq, (new_dist, (new_row, new_col)))
                    parent[new_row][new_col] = (curr_row, curr_col)

    path = []
    row, col = destination

    if distance[row][col] == INF:
        print("Er is geen pad naar de bestemming.")
        return path

    while row != -1 and col != -1:
        path.insert(0, (row, col))
        parent_row, parent_col = parent[row][col]
        row, col = parent_row, parent_col

    return path

def update_grid(grid, row, col, new_value):
    grid[row][col] = new_value

#De functies location direction en sensor_values kunnen 1 functie worden.
def location(msg,robot):
    global client
    number_robot = int(msg[10])
    location = [round(round(web_robots[number_robot].getField('translation').getSFVec3f()[0],1)*10), round(round(web_robots[number_robot].getField('translation').getSFVec3f()[1],1)*10)]
    print(f"The location of `{robot}` is x : {location[0]} and y : {location[1]}")
    
    msg = (msg+f":({location[0]},{location[1]})")
    result = client.publish(topic_server, msg)
    status = result[0]
    if status == 0:
        print(f'Send `{msg}` to topic `{topic_server}`')
    else:
        print(f'Failed to send message to topic {topic_server}')
        return False
        
def direction(msg,robot):
    global client
    number_robot = int(msg[10])
    direction = web_robots[number_robot].getField('direction_lamps').getSFString()
    print(f"The direction of `{robot}` is x : {direction}")
    msg = (msg+f":{direction}")
    result = client.publish(topic_server, msg)
    status = result[0]
    if status == 0:
        print(f'Send `{msg}` to topic `{topic_server}`')
    else:
        print(f'Failed to send message to topic {topic_server}')
        return False
        
def sensor_value(msg,robot):
    global client
    number_robot = int(msg[10])
    sensor_values = [web_robots[number_robot].getField('sensors').getMFInt32(0), web_robots[number_robot].getField('sensors').getMFInt32(1),web_robots[number_robot].getField('sensors').getMFInt32(2),web_robots[number_robot].getField('sensors').getMFInt32(3)]     
    print(f"The sensor values of `{robot}` is x : {sensor_values}")
    msg = (msg+f":{sensor_values}")
    result = client.publish(topic_server, msg)
    status = result[0]
    if status == 0:
        print(f'Send `{msg}` to topic `{topic_server}`')
    else:
        print(f'Failed to send message to topic {topic_server}')
        return False

#Options are "web_robot_x:emgc:on_" and "web_robot_x:emgc:off"
def emergency(msg,robot):

    value_emergency = msg[17:20]
    if(value_emergency == "on"):
        print(f"Emergency button has been pushed by `{robot}`")
        for web_robot in web_robots:
            web_robot.getField('emergency').setSFBool(True)
    elif(value_emergency == "off" and robot == "dsh_board_0"):
        print(f"Emergency button has been reset by `{robot}`")
        for web_robot in web_robots:
            web_robot.getField('emergency').setSFBool(False)
    else:
        print(f"Error while parsing command : `{msg}` from `{robot}`") 

def target(msg,robot):
    #Wat als de gebruiker -1 of 10 doorstuurt. Ik lees het dan uit als 1 of 0 etc
    valid_command = False
    try:
        number_robot = int(msg[10])
        target_robot_x = int(msg[18:19])
        target_robot_y = int(msg[20:21])
        valid_command = True
    except:
        print(f"Error while parsing command : `{msg}` from `{robot}`") 

    if(valid_command and len(msg) > 21):
        location = [round(round(web_robots[number_robot].getField('translation').getSFVec3f()[0],1)*10), round(round(web_robots[number_robot].getField('translation').getSFVec3f()[1],1)*10)]
        source = (location[0], location[1])          # Startknooppunt (S)
        destination = (target_robot_x, target_robot_y)     # Bestemmingsknooppunt (D)

        for target in dijkstra(grid, source, destination):
            tarX = int(target[0])
            tarY = int(target[1])
            put_queue(number_robot,tarX,tarY)

        update_grid(grid, target_robot_x, target_robot_y, INF)
        # target = [target_robot_x,target_robot_y]
        # web_robots[number_robot].getField('target').setSFVec2f(target)

def put_queue(robot,robot_x,robot_y):
    global id
    id += 1
    queue = queues[robot]
    queue.put([robot_x,robot_y,id])  
    queue_size = queue.qsize()
    msg = f"add: web_robot_{robot}:targ:({queue.queue[queue_size-1][0]},{queue.queue[queue_size-1][1]}):id:{queue.queue[queue_size-1][2]}"
    client.publish(topic_server, msg) 
    print(f"Added target ({robot_x},{robot_y}) to the queue from web_robot_{robot} with id `{id}`") 
    if(id == 1000):
        id = 0

def remove_queue(client,queue,robot):
    global id
    msg = f"remove: web_robot_{robot}:targ:({queue.queue[0][0]},{queue.queue[0][1]}):id:{queue.queue[0][2]}"
    client.publish(topic_server, msg)
    print(f"Removed target ({queue.queue[0][0]},{queue.queue[0][1]}) from the queue from web_robot_{robot} with id `{queue.queue[0][2]}`") 
    removed_queue = queue.get()
         

def check_pixels(client):
    message = 'run pixels!!!'
    print(message)
    q=0
    # Get the camera image
    image = camera.getImage()

    for y in range(height):
        for x in range(width):
            # Calculate the index of the pixel in the image buffer
            index = (y * width + x) * 4

            # Get the RGB values of the pixel
            b = image[index]
            g = image[index + 1]
            r = image[index + 2]
            
                        # Print the RGB values
            # print(f"Pixel ({x}, {y}): R={r}, G={g}, B={b}")
            msg = f"pixel_color:({x},{y}):({r},{g},{b})"
            client.publish(topic_server, msg)
            
            if r < 50 and g < 50 and b < 50:
                print(f"opstakel {x} {y}")
                for row in grid:
                    print(' '.join(map(str, row)))
                update_grid(grid, y, x, INF)
            else: 
                update_grid(grid, y, x, 1)
            if r > 200 and g > 200 and b < 100:
                update_grid(grid, y, x, 5)
   
   
            q += 1
    

commands_web = ["loca","targ","dire","sens"]
commands_esp = ["emgc"]
commands_dsh = ["emgc"]
web_robots_str = ["web_robot_0","web_robot_1","web_robot_2","web_robot_3"]
esp_robots_str = ["esp_robot_0","esp_robot_1","esp_robot_2","esp_robot_3"]
dsh_boards_str =  ["dsh_board_0"]


functions = {
             'loca':location,
             'targ':target,
             'dire':direction,
             'sens':sensor_value,
             'emgc':emergency
            }

def connect_mqtt():
   
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic_robots)
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    return client

def on_disconnect(client, userdata, rc):
    logging.info("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        logging.info("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.info("Reconnected successfully!")
            return
        except Exception as err:
            logging.error("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    logging.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)
    global FLAG_EXIT
    FLAG_EXIT = True


def publish(client):



    if not client.is_connected():
        print("publish: MQTT client is not connected!")
        return False
    
    send_pixel_colors = False
    i = 0
    
    for web_robot in web_robots:
        location = [round(round(web_robot.getField('translation').getSFVec3f()[0],1)*10), round(round(web_robot.getField('translation').getSFVec3f()[1],1)*10)]
        direction = web_robot.getField('direction_lamps').getSFString()
        sensor_values = [web_robot.getField('sensors').getMFInt32(0), web_robot.getField('sensors').getMFInt32(1),web_robot.getField('sensors').getMFInt32(2),web_robot.getField('sensors').getMFInt32(3)]
        if(location[0] == Old_locations[i][0] and location[1] == Old_locations[i][1]):
            pass
        else:
            msg = f"{web_robots_str[i]}:loca:({location[0]},{location[1]})"
            result = client.publish(topic_server, msg)
            send_pixel_colors = True
            # print("send_pixel_colors = " + str(send_pixel_colors))
            status = result[0]
            if status == 0:
                print(f'Send `{msg}` to topic `{topic_server}`')
            else:
                print(f'Failed to send message to topic {topic_server}')
                return False
                
        Old_locations[i][0] = location[0]
        Old_locations[i][1] = location[1]
        
        if(direction == Old_directions[i] and direction == Old_directions[i]):
            pass
        else:
            if direction not in direction_options:
                print(f"{web_robots_str[i]} with invalid direction : {direction}")
            else:
                msg = f"{web_robots_str[i]}:dire:{direction}"
                result = client.publish(topic_server, msg)
                send_pixel_colors = True
                status = result[0]
                if status == 0:
                    print(f'Send `{msg}` to topic `{topic_server}`')
                else:
                    print(f'Failed to send message to topic {topic_server}')
                    return False
                    
        Old_directions[i] = direction

        
        j = 0
        sensor_has_different_values = False
        for value in Old_sensor_values:
            if(Old_sensor_values[i][j] != sensor_values[j]):
                sensor_has_different_values = True
                break
            j += 1    
           
        if(sensor_has_different_values):
            msg = f"{web_robots_str[i]}:sens:{sensor_values}"
            result = client.publish(topic_server, msg)
            send_pixel_colors = True
            status = result[0]
            if status == 0:
                print(f'Send `{msg}` to topic `{topic_server}`')
            else:
                print(f'Failed to send message to topic {topic_server}')
                return False
        
        Old_sensor_values[i][0] = sensor_values[0]
        Old_sensor_values[i][1] = sensor_values[1]
        Old_sensor_values[i][2] = sensor_values[2]
        Old_sensor_values[i][3] = sensor_values[3]
        
        queue = queues[i]
      
       
        if(not queue.empty()):
            web_robots[i].getField('target').setSFVec2f([queue.queue[0][0],queue.queue[0][1]])
            if(location[0] == queue.queue[0][0] and location[1] == queue.queue[0][1]):
                remove_queue(client,queue,i)
        
        
             
        
        if(i == 3 and send_pixel_colors):
            check_pixels(client)
            return True
        elif(i == 3):
            return True
        i += 1
        
    return True


def on_message(client, userdata, msg):
    incoming = msg.payload.decode()
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
    pick_up_robot = None
    incoming_robot = incoming[:11]
    check_pixels(client)
    
    if incoming_robot == "pixel_color":
        check_pixels(client)
    elif incoming_robot in web_robots_str:
        print(f"This is robot : `{incoming_robot}`")
        pick_up_robot = incoming_robot
    elif incoming_robot in esp_robots_str:
        print(f"This is robot : `{incoming_robot}`")
        pick_up_robot = incoming_robot
    elif incoming_robot in dsh_boards_str:
        print(f"This is dashboard : `{incoming_robot}`")
        pick_up_robot = incoming_robot
    else:
        print(f"Incoming robot `{incoming}` not recognized")
        return
    
    incoming_command = incoming[12:16]
    if incoming_command in commands_web and pick_up_robot[0:3] == "web":
        print(f"{pick_up_robot} sent command `{incoming_command}`")
        functions[incoming_command](incoming,pick_up_robot)
    elif incoming_command in commands_esp and pick_up_robot[0:3] == "esp":
        print(f"{pick_up_robot} sent command `{incoming_command}`")
        functions[commands_esp[0]](incoming,pick_up_robot)
    elif incoming_command in commands_dsh and pick_up_robot[0:3] == "dsh":
        print(f"{pick_up_robot} sent command `{incoming_command}`")
        functions[incoming_command](incoming,pick_up_robot)
    elif incoming_command in commands_web and (pick_up_robot[0:3] == "esp" or pick_up_robot[0:3] == "dsh") or incoming_command in commands_esp and pick_up_robot[0:3] == "web" :
        print(f"{pick_up_robot} sent command `{incoming_command}` but has no rights to use this commands")
    else:
        print(f"Incoming command `{incoming_command}` not recognized")
        return




def run():
    i = 0
    global client
    client = connect_mqtt()
    client.loop_start()
    time.sleep(1)
    timer = 0
    while supervisor.step(64) != -1:
        if timer == 960:
            publish(client)
            timer = 0
        timer += 64
        pass

if __name__ == '__main__':
    run()