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


#Get robots in webots
web_robot_0 = supervisor.getFromDef("pick_up_robot_0")
web_robot_1 = supervisor.getFromDef("pick_up_robot_1")
web_robot_2 = supervisor.getFromDef("pick_up_robot_2")
web_robot_3 = supervisor.getFromDef("pick_up_robot_3")

web_robots = [web_robot_0,web_robot_1,web_robot_2,web_robot_3]

#Kan een lengte van 2 hebben in plaats van 3 
Old_locations = [[-1,-1,-1],[-1,-1,-1],[-1,-1,-1],[-1,-1,-1]]

locations_grid = [[-1,-1],[-1,-1],[-1,-1],[-1,-1]]
next_locations = [[-1,-1],[-1,-1],[-1,-1],[-1,-1]]

Old_directions = ["off","off","off","off"]
direction_options = ["off","forward","backwards","left","right"]

Old_sensor_values = [[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1]]

#Credentials for MQTT
broker = 'localhost'
port = 1883
topic_server = "publish/server"
topic_robots = "publish/robots"
topic_pixels = "publish/pixels"
client_id = f'python-mqtt-{random.randint(0, 1000)}'

#Queue for the next step for the web_robot
q_w0 = Queue()
q_w1 = Queue()
q_w2 = Queue()
q_w3 = Queue()

queues = [q_w0,q_w1,q_w2,q_w3]

#Queue for the next task to complete at a certain location
targ0 = Queue()
targ1 = Queue()
targ2 = Queue()
targ3 = Queue()

targets = [targ0,targ1,targ2,targ3]

#id of the messages
id = 0

emergency_is_false = False

client = None

#Status of the emergency button 
emgc_on = False

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

FLAG_EXIT = False

INF = float('inf')

import heapq
from typing import List, Tuple

INF_ROBOT = 1000

#Dijkstra algoritm 
#Parameters are : grid, source location, destination location
#Gives back a grid with the new path
# def dijkstra(grid: List[List[int]], source: Tuple[int, int], destination: Tuple[int, int]):
    # rows = len(grid)
    # cols = len(grid[0])

    # distance = [[INF] * cols for _ in range(rows)]
    # visited = [[False] * cols for _ in range(rows)]
    # parent = [[(-1, -1)] * cols for _ in range(rows)]

    # dr = [0, 0, 1, -1]
    # dc = [1, -1, 0, 0]

    # def is_valid(row: int, col: int) -> bool:
        # return 0 <= row < rows and 0 <= col < cols

    # def get_weight(row: int, col: int) -> int:
        # return grid[row][col]

    # pq = []
    # heapq.heappush(pq, (0, source))
    # distance[source[0]][source[1]] = 0

    # while pq:
        # dist, current = heapq.heappop(pq)
        # curr_row, curr_col = current

        # if visited[curr_row][curr_col]:
            # continue

        # visited[curr_row][curr_col] = True

        # if current == destination:
            # break

        # for i in range(4):
            # new_row = curr_row + dr[i]
            # new_col = curr_col + dc[i]

            # if is_valid(new_row, new_col):
                # weight = get_weight(new_row, new_col)
                # new_dist = distance[curr_row][curr_col] + weight

                # if new_dist < distance[new_row][new_col]:
                    # distance[new_row][new_col] = new_dist
                    # heapq.heappush(pq, (new_dist, (new_row, new_col)))
                    # parent[new_row][new_col] = (curr_row, curr_col)

    # path = []
    # row, col = destination

    # if distance[row][col] == INF:
    # if(distance[row][col] == INF):
        # print("Er is geen pad naar de bestemming.")
        # return path

    # while row != -1 and col != -1:
        # path.insert(0, (row, col))
        # parent_row, parent_col = parent[row][col]
        # row, col = parent_row, parent_col

    # return path

#Update the grid
# def update_grid(grid, row, col, new_value):
    # grid[row][col] = new_value

#Get the location of a robot and send
#a message on the topic publish/server
# def location(msg,robot):
    # global client
    # number_robot = int(msg[10])
    # location = [round(round(web_robots[number_robot].getField('translation').getSFVec3f()[0],1)*10), round(round(web_robots[number_robot].getField('translation').getSFVec3f()[1],1)*10)]
    # print(f"The location of `{robot}` is x : {location[0]} and y : {location[1]}")
    
    # msg = (msg+f":({location[0]},{location[1]})")
    # result = client.publish(topic_server, msg)
    # status = result[0]
    # if status == 0:
        # print(f'Send `{msg}` to topic `{topic_server}`')
    # else:
        # print(f'Failed to send message to topic {topic_server}')
        # return False
  
#Get the direction of a robot and send
#a message on the topic publish/server 
# def direction(msg,robot):
    # global client
    # number_robot = int(msg[10])
    # direction = web_robots[number_robot].getField('direction_lamps').getSFString()
    # print(f"The direction of `{robot}` is x : {direction}")
    # msg = (msg+f":{direction}")
    # result = client.publish(topic_server, msg)
    # status = result[0]
    # if status == 0:
        # print(f'Send `{msg}` to topic `{topic_server}`')
    # else:
        # print(f'Failed to send message to topic {topic_server}')
        # return False
        
#Get sensor data of a robot and send
#a message on the topic publish/server
# def sensor_value(msg,robot):
    # global client
    # number_robot = int(msg[10])
    # sensor_values = [web_robots[number_robot].getField('sensors').getMFInt32(0), web_robots[number_robot].getField('sensors').getMFInt32(1),web_robots[number_robot].getField('sensors').getMFInt32(2),web_robots[number_robot].getField('sensors').getMFInt32(3)]     
    # print(f"The sensor values of `{robot}` is x : {sensor_values}")
    # msg = (msg+f":{sensor_values}")
    # result = client.publish(topic_server, msg)
    # status = result[0]
    # if status == 0:
        # print(f'Send `{msg}` to topic `{topic_server}`')
    # else:
        # print(f'Failed to send message to topic {topic_server}')
        # return False

#Set a target for a robot through the algoritm of Dijkstra

# def target(msg,robot):
    # global id
    # valid_command = False
    # try:
        # number_robot = int(msg[10])
        # target_robot_x = int(msg[18:19])
        # target_robot_y = int(msg[20:21])
        # valid_command = True
    # except:
        # print(f"Error while parsing command : `{msg}` from `{robot}`") 

    # if(valid_command and len(msg) > 21):
        # location = [round(round(web_robots[number_robot].getField('translation').getSFVec3f()[0],1)*10), round(round(web_robots[number_robot].getField('translation').getSFVec3f()[1],1)*10)]
        # source = (location[0], location[1])          # Startknooppunt (S)
        # destination = (target_robot_x, target_robot_y)     # Bestemmingsknooppunt (D)

        # if(source != destination):
        
            # doel = dijkstra(grid, source, destination)
            # if(not doel == []):
                # next_step_x = doel[1][0]
                # next_step_y = doel[1][1]
                # put_target(number_robot,next_step_x,next_step_y,target_robot_x,target_robot_y)
                   
            # else:
                # print(f"web_robot_{number_robot} kan niet naar locatie ({target_robot_x},{target_robot_y}) gaan want daar is een obstakel")
        # else:
            # print(f"web_robot_{number_robot} is al op locatie ({target_robot_x},{target_robot_y})")

#Set a target to the queue of a certain robot
# def put_target(robot,next_x,next_y,targ_x,targ_y):
    # global id
    # global targets
    # id += 1
    # target = targets[robot]
    # target.put([targ_x,targ_y,id])
    # msg = f"add:web_robot_{robot}:targ:({targ_x},{targ_y}):id:{id}"   
    # client.publish(topic_server, msg) 
    # print(f"Added target ({targ_x},{targ_y}) to the queue from web_robot_{robot} with id `{id}`") 
    # if(id == 1000):
        # id = 0
 

#Check the pixels that the camera sees
#subsequently send it to MQTT on the publish/server topic
	
def check_pixels(client):
    # message = 'run pixels!!!'
    # print(message)

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
            client.publish(topic_pixels, msg)
            
            
            if r < 50 and g < 50 and b < 50:

                update_grid(grid, y, x, INF)
            else:
                update_grid(grid, y, x, 1)

            q += 1
    i = 0
    for web_robot in web_robots:
        update_grid(grid,locations_grid[i][0],locations_grid[i][1], INF)
        i += 1

#Commands that everyone can use
#This is not up to date
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

#Connect to mqtt
def connect_mqtt():
   
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(topic_robots)
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    return client

#Try to reconnect to the MQTT server if no connection can be established
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

#Publish the data if it changes to the topic publish/server
def publish(client):
    check_pixels(client)
    global grid
    global locations_grid


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

            update_grid(grid, location[0], location[1], INF)
            pass
        else:
            msg = f"{web_robots_str[i]}:loca:({location[0]},{location[1]})"
            result = client.publish(topic_server, msg)
            send_pixel_colors = True
            update_grid(grid, Old_locations[i][0], Old_locations[i][1], 1)
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
            queue.get()
            
 
        target = targets[i]
        if(not target.empty()):
 
            location = [round(round(web_robot.getField('translation').getSFVec3f()[0],1)*10), round(round(web_robot.getField('translation').getSFVec3f()[1],1)*10)]  
            # for g in grid:
                # print(g)
            source = (location[0], location[1])          # Startknooppunt (S)
            destination = (targets[i].queue[0][0], targets[i].queue[0][1]) 

            # print("Source = " + str(source))
            # print("Destination = " + str(destination)) 
            
            if(source != destination):
                 # print("Source is niet gelijk aan destination")
                 doel = dijkstra(grid, source, destination)
                 # print(doel)
                 if(not doel == []):
                     next_step_x = doel[1][0]
                     next_step_y = doel[1][1]
                     queue.put([next_step_x,next_step_y,i])
                     update_grid(grid, next_step_x, next_step_y, INF)
                     # next_locations[i][0] = next_step_x
                     # next_locations[i][1] = next_step_y
                 
            else:
                 remove_queue(client,targets[i],i)
                 
            
            locations_grid[i][0] = location[0]
            locations_grid[i][1] = location[1]
            
            # print(f"Location_grid = ({locations_grid[i][0]},{locations_grid[i][1]})")
        
        if(i == 3 and send_pixel_colors):
            check_pixels(client)
            update_grid_for_robots()
            return True
        elif(i == 3):
            update_grid_for_robots()
            return True
        i += 1
    

    return True

def update_grid_for_robots():
    i = 0
    for web_robot in web_robots:
        location = [round(round(web_robot.getField('translation').getSFVec3f()[0],1)*10), round(round(web_robot.getField('translation').getSFVec3f()[1],1)*10)]
        update_grid(grid, Old_locations[i][0], Old_locations[i][1], 1)
        update_grid(grid, location[0], location[1], INF)
        # update_grid(grid, next_locations[i][0], next_locations[i][1], INF)
        i += 1

#If there is a message on publish/robots the person is authorized if it
#is not a get message. If it can not be proccesed an error message will
#be displayed in the terminal

def on_message(client, userdata, msg):
    incoming = msg.payload.decode()
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
    pick_up_robot = None
    incoming_robot = incoming[:11]
    
    
    if incoming_robot == "pixel_color":
        check_pixels(client)
    elif incoming_robot == "emgc_status":
        emgc_status(client)
    elif incoming[:12] == "remove_queue":
        remove_queue_with_id(client,incoming)
    elif incoming[:10] == "get_queues":
        get_queues(client)
    else:
        if incoming_robot in web_robots_str:
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
            functions[commands_esp[0]](incoming,pick_up_robot,client)
        elif incoming_command in commands_dsh and pick_up_robot[0:3] == "dsh":
            print(f"{pick_up_robot} sent command `{incoming_command}`")
            functions[incoming_command](incoming,pick_up_robot,client)
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
    config_web_robots(client)
    while supervisor.step(64) != -1:
        if timer == 960:
            publish(client)
            timer = 0
        timer += 64
        pass

if __name__ == '__main__':
    run()