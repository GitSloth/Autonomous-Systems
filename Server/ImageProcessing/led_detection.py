import cv2
import numpy as np
from Camera import Camera

# Function to do nothing
def nothing(x):
    pass

# Capture 
cam = Camera(Camera.CAMERA, source=0)

#create windows
cv2.namedWindow('Settings', cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow('Settings', 450,550)
cv2.namedWindow('Output', cv2.WINDOW_AUTOSIZE)
#cv2.namedWindow('Contours', cv2.WINDOW_AUTOSIZE)
#cv2.namedWindow('masked', cv2.WINDOW_NORMAL)
#cv2.namedWindow('bright', cv2.WINDOW_NORMAL)

# Initial HSV values for green and red
green_low_h, green_high_h = 40, 80
green_low_s, green_high_s = 100, 255
green_low_v, green_high_v = 100, 255

red_low_h_1, red_high_h_1 = 0, 10
red_low_h_2, red_high_h_2 = 160, 179
red_low_s, red_high_s = 100, 255
red_low_v, red_high_v = 20, 255

# Callback functions to update the trackbar values
def update_green_low_h(val):
    global green_low_h
    green_low_h = val
def update_green_high_h(val):
    global green_high_h
    green_high_h = val
def update_green_low_s(val):
    global green_low_s
    green_low_s = val
def update_green_high_s(val):
    global green_high_s
    green_high_s = val
def update_green_low_v(val):
    global green_low_v
    green_low_v = val
def update_green_high_v(val):
    global green_high_v
    green_high_v = val
def update_red_low_h1(val):
    global red_low_h_1
    red_low_h_1 = val
def update_red_high_h1(val):
    global red_high_h_1
    red_high_h_1 = val
def update_red_low_h2(val):
    global red_low_h_2
    red_low_h_2 = val
def update_red_high_h2(val):
    global red_high_h_2
    red_high_h_2 = val
def update_red_low_s(val):
    global red_low_s
    red_low_s = val
def update_red_high_s(val):
    global red_high_s
    red_high_s = val
def update_red_low_v(val):
    global red_low_v
    red_low_v = val
def update_red_high_v(val):
    global red_high_v
    red_high_v = val

# Create windows
cv2.namedWindow('Settings', cv2.WINDOW_AUTOSIZE)

# Create trackbars and set callback functions
cv2.createTrackbar('green l H', 'Settings', green_low_h, 255, update_green_low_h)
cv2.createTrackbar('green h H', 'Settings', green_high_h, 255, update_green_high_h)
cv2.createTrackbar('green l S', 'Settings', green_low_s, 255, update_green_low_s)
cv2.createTrackbar('green h S', 'Settings', green_high_s, 255, update_green_high_s)
cv2.createTrackbar('green l V', 'Settings', green_low_v, 255, update_green_low_v)
cv2.createTrackbar('green h V', 'Settings', green_high_v, 255, update_green_high_v)

cv2.createTrackbar('red low H1', 'Settings', red_low_h_1, 255, update_red_low_h1)
cv2.createTrackbar('red high H1', 'Settings', red_high_h_1, 255, update_red_high_h1)
cv2.createTrackbar('red low H2', 'Settings', red_low_h_2, 255, update_red_low_h2)
cv2.createTrackbar('red high H2', 'Settings', red_high_h_2, 255, update_red_high_h2)
cv2.createTrackbar('red low S', 'Settings', red_low_s, 255, update_red_low_s)
cv2.createTrackbar('red high S', 'Settings', red_high_s, 255, update_red_high_s)
cv2.createTrackbar('red low V', 'Settings', red_low_v, 255, update_red_low_v)
cv2.createTrackbar('red high V', 'Settings', red_high_v, 255, update_red_high_v)

while(True):
    #get img from camera and turn into hsv image
    img = cam.getImage()
    #cv2.resize(img, (1280,720), interpolation=cv2.INTER_AREA)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #create the lower and upper limits
    #red, has 2 ranges 
    lower_hsv_red_1 = np.array([red_low_h_1, red_low_s, red_low_v])
    higher_hsv_red_1 = np.array([red_high_h_1,red_high_s,red_high_v])

    lower_hsv_red_2 = np.array([red_low_h_2, red_low_s, red_low_v])
    higher_hsv_red_2 = np.array([red_high_h_2,red_high_s,red_high_v])

    #white
    lower_hsv_white = np.array([0, 0, 168])
    higher_hsv_white = np.array([172,111,255])

    #green
    lower_hsv_green = np.array([green_low_h, green_low_s, green_low_v])
    higher_hsv_green = np.array([green_high_h,green_high_s,green_high_v])

    #bright spots
    blurred = cv2.GaussianBlur(gray_img, (11,11), 0)
    _, thresh = cv2.threshold(gray_img, 240, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_led = None
    red_led = None
    for contour in contours:
        if cv2.contourArea(contour) < 10:
            continue
        x, y, w, h  = cv2.boundingRect(contour)
        roi = hsv_img[y:y+h,x:x+h]
        #create masks
        sub_mask_red_1 = cv2.inRange(roi, lower_hsv_red_1,higher_hsv_red_1)
        sub_mask_red_2 = cv2.inRange(roi, lower_hsv_red_2,higher_hsv_red_2)
        mask_red = sub_mask_red_1+sub_mask_red_2
        mask_green = cv2.inRange(roi, lower_hsv_green, higher_hsv_green)

        if cv2.countNonZero(mask_green) > 0 and green_led is None:
            green_led = (x, y, w, h, 'green')
        # Check if red LED is present
        elif cv2.countNonZero(mask_red) > 0 and red_led is None:
            red_led = (x, y, w, h, 'red')
    
        # Break the loop if both LEDs are found
        if green_led is not None and red_led is not None:
            break

    # Draw rectangles around the LEDs
    if green_led is not None:
        x, y, w, h, color = green_led
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    if red_led is not None:
        x, y, w, h, color = red_led
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)

    #cv2.imshow('contours', frame_with_contours)
    cv2.imshow('Output', img)
    #cv2.imshow('masked', masked_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cam.release()
cv2.destroyAllWindows()
