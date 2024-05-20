'''
Useful starting point to setup the work environment
https://randomnerdtutorials.com/getting-started-raspberry-pi-pico-w/

This code exposes a small webpage allowing to tunr on or off the front (Red) and back (Green) leds and allowing to move
the two wheels forward (supposedly) for 1 second at moderate speed. To be fully tested.

Upload both this file and main.html on the picoW
To test in your own house, change the ssid and password accordingly
'''


import socket
import network
import time
from machine import Pin, PWM, ADC


BUILT_IN_LED=25 # Built in led
FLED=20 # Front led Red
BLED=21 # Back led Green
PWM_LM=6 # Left Continuous Servo
PWM_RM=7 # Right Continuous Servo
PWM_SC=10 # Panning Servo
SDA=4
SCL=5
MISO=16
MOSI=19
SCK=18
CS=17
LDR_PIN = 27

# insert here your network parameters
#ssid=b'tesla iot'
#pwd=b'fsL6HgjN'

ssid=b'TP-LINK_F1F6'
pwd=b'18930160'


# initial state definition
built_in_led = machine.Pin("LED", machine.Pin.OUT) # build in led
fled = Pin(FLED, Pin.OUT) # front led
bled = Pin(BLED, Pin.OUT) # back led
fled.value(True)
bled.value(False) 
built_in_led.value(True)
time.sleep(1)
built_in_led.value(False)
time.sleep(1)
fled.value(False)

#setus up servos
LeftMotor = PWM(Pin(PWM_LM))
LeftMotor.freq(50)
RightMotor = PWM(Pin(PWM_RM))
RightMotor.freq(50)
PanMotor = PWM(Pin(PWM_SC))
PanMotor.freq(50)

# Initialize ADC for LDR
ldr = ADC(Pin(LDR_PIN))

with open("main.html", "r") as page:
    html = page.read()

# function controlling servos
def MoveForward(power,Stime):
     LeftMotor.duty_u16(2000)
     RightMotor.duty_u16(6600)
     time.sleep(Stime)
     LeftMotor.duty_u16(5000)
     RightMotor.duty_u16(5000)
     SpinTop()
     
def MoveBack(power,Stime):
     LeftMotor.duty_u16(6600)
     RightMotor.duty_u16(2000)
     time.sleep(Stime)
     LeftMotor.duty_u16(5000)
     RightMotor.duty_u16(5000)
     SpinTop()
     
def SpinLeft(power,Stime):
     LeftMotor.duty_u16(5000)
     RightMotor.duty_u16(6000)
     time.sleep(Stime)
     LeftMotor.duty_u16(5000)
     RightMotor.duty_u16(5000)
     SpinTop()
    
def SpinRight(power,Stime):
     LeftMotor.duty_u16(2000)
     RightMotor.duty_u16(5000)
     time.sleep(Stime)
     LeftMotor.duty_u16(5000)
     RightMotor.duty_u16(5000)
     SpinTop()
     
def SpinTop():
    
    duty_cycles = [2000, 3000, 4000, 5000, 6000]

    for duty in duty_cycles:
        PanMotor.duty_u16(duty)  # Rotate the servo to the specified angle
        time.sleep(0.5)  # Wait for the servo to settle
        print("LDR Value:", ldr.read_u16())  # Print LDR value
        time.sleep(1)  # Pause for 1 second

    # Stop the servo at the end
    PanMotor.duty_u16(5000)  # Stop the servo (neutral position)
    print("LDR Value:", ldr.read_u16())  # Print LDR value at the end
    time.sleep(1)  # Pause for 1 second
    
    #PanMotor.duty_u16(8000)  # Rotate the servo slightly clockwise
    #PanMotor.duty_u16(9000)


# activate the Pico Lan
network.hostname("mypicow") #wlan.config(hostname="mypico")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Hostname set to: "+str(network.hostname()))

time0=time.time()
wlan.connect(ssid, pwd)
while 1:
    if(wlan.isconnected()):
        print("\nConnected!\n")
        built_in_led.value(True)
        break
    else:
        print(".")
        time.sleep(1)
        if(time.time()-time0>10):
            print("Connection could not be established")
            break

sta_if = network.WLAN(network.STA_IF)

print(sta_if.ifconfig()[0]) # prints the IP on the serial

# listen on port 80
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(addr)
print("Listening to port 80\n")
s.listen(1)

while True:
    cl, addr = s.accept()
    print("Incoming connection request from: "+str(addr)+"\n")
    # here is the place where we get the request body...
    cl_file = cl.makefile('rwb', 0)
    found=False
    while True:
        line = cl_file.readline()
        if not line or line == b'\r\n':
           break
        if not found: 
            if str(line).find("/?PRESS=FRONT_LED_ON") !=-1:
                #print("Command_1 ON received")
                fled.value(True)
                found=True
            if str(line).find("/?PRESS_1=FRONT_LED_OFF") !=-1:
                #print("Command_1 OFF received")
                fled.value(False)
                found=True
            if str(line).find("/?PRESS_2=BACK_LED_ON") !=-1:
                #print("Command_2 ON received")
                bled.value(True)
                found=True
            if str(line).find("/?PRESS_3=BACK_LED_OFF") !=-1:
                #print("Command_2 OFF received")
                bled.value(False)
                found=True
            if str(line).find("/?PRESS_4=MOVE_FORWARD") !=-1:
                #print("Command MOVE_FORWARD received")
                MoveForward(50,1)
                found=True
            if str(line).find("/?PRESS_5=MOVE_BACK") !=-1:
                #print("Command MOVE_BACK received")
                MoveBack(50,1)
                found=True
            if str(line).find("/?PRESS_6=SPIN_LEFT") !=-1:
                #print("Command SPIN_LEFT received")
                SpinLeft(50,1)
                found=True
            if str(line).find("/?PRESS_7=SPIN_RIGHT") !=-1:
                #print("Command SPIN_RIGHT received")
                SpinRight(50,1)
                found=True
            if str(line).find("/?PRESS_8=SPIN_TOP") !=-1:
                #print("Command SPIN_TOP received")
                SpinTop(50,1)
                found=True
       
# we process the response file, We can add placeholders to turn change the page aspect
    response=html # default page, placeholders needs to be replaced before submitting
    # send the page
    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(response)
    cl.close()
    

