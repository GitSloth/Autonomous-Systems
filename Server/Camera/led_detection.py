import cv2
import numpy as np

# Function to do nothing
def nothing(x):
    pass

VIDEO = False

# Capture 
if VIDEO:
    cap = cv2.VideoCapture('http://192.168.137.224:8080/video') # ip phone camera stream
    if not cap.isOpened():
        print("Error: Could not open video source.")
        exit()
else:
    image = cv2.imread('test_image_higher.png')
    if image is None:
        print("Error: Could not load the image.")
        exit()

#create windows
cv2.namedWindow('Settings', cv2.WINDOW_NORMAL)
cv2.namedWindow('contours', cv2.WINDOW_NORMAL)
cv2.namedWindow('bright', cv2.WINDOW_NORMAL)
# color sliders
cv2.createTrackbar('green l H', 'Settings', 40, 179, nothing)
cv2.createTrackbar('green h H', 'Settings', 80, 179, nothing)
cv2.createTrackbar('green l S', 'Settings', 100, 255, nothing)
cv2.createTrackbar('green h S', 'Settings', 255, 255, nothing)
cv2.createTrackbar('green l V', 'Settings', 100, 255, nothing)
cv2.createTrackbar('green h V', 'Settings', 255, 255, nothing)

cv2.createTrackbar('red low H', 'Settings', 0, 179, nothing)
cv2.createTrackbar('red high H', 'Settings', 10, 179, nothing)
cv2.createTrackbar('red low S', 'Settings', 100, 255, nothing)
cv2.createTrackbar('red high S', 'Settings', 255, 255, nothing)
cv2.createTrackbar('red low V', 'Settings', 165, 255, nothing)
cv2.createTrackbar('red high V', 'Settings', 255, 255, nothing)

while(True):
    #get frame from video or read image and turn into hsv image
    if VIDEO:
        ret, frame = cap.read()
        if not ret:
            print("failed to cap frame")
            break
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #get the settings values
    green_low_h = cv2.getTrackbarPos('green l H', 'Settings')
    green_high_h = cv2.getTrackbarPos('green h H', 'Settings')
    green_low_s = cv2.getTrackbarPos('green l S', 'Settings')
    green_high_s = cv2.getTrackbarPos('green h S', 'Settings')
    green_low_v = cv2.getTrackbarPos('green l V', 'Settings')
    green_high_v = cv2.getTrackbarPos('green h V', 'Settings')

    red_low_h = cv2.getTrackbarPos('red low H', 'Settings')
    red_high_h = cv2.getTrackbarPos('red high H', 'Settings')
    red_low_s = cv2.getTrackbarPos('red low S', 'Settings')
    red_high_s = cv2.getTrackbarPos('red high S', 'Settings')
    red_low_v = cv2.getTrackbarPos('red low V', 'Settings')
    red_high_v = cv2.getTrackbarPos('red high V', 'Settings')
    
    #create the lower and upper limits
    lower_hsv_red = np.array([red_low_h, red_low_s, red_low_v])
    higher_hsv_red = np.array([red_high_h,red_high_s,red_high_v])

    lower_hsv_white = np.array([0, 0, 168])
    higher_hsv_white = np.array([172,111,255])

    lower_hsv_green = np.array([green_low_h, green_low_s, green_low_v])
    higher_hsv_green = np.array([green_high_h,green_high_s,green_high_v])

    #mask_white = cv2.inRange(hsv_frame, lower_hsv_white,higher_hsv_white)
    #create masks
    if VIDEO:
        mask_red = cv2.inRange(hsv_frame, lower_hsv_red,higher_hsv_red)
        mask_green = cv2.inRange(hsv_frame, lower_hsv_green, higher_hsv_green)
        
    else:
        mask_red = cv2.inRange(hsv_image, lower_hsv_red,higher_hsv_red)
        mask_green = cv2.inRange(hsv_image, lower_hsv_green, higher_hsv_green)

    # some filters, supposedly increasing quality. Not tested
    mask_green = cv2.erode(mask_green, None, iterations=2)
    mask_green = cv2.dilate(mask_green, None, iterations=2)
    mask_red = cv2.erode(mask_red, None, iterations=2)
    mask_red = cv2.dilate(mask_red, None, iterations=2)
    mask =  mask_green + mask_red

    # Find contours for green and red LEDs
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on size and shape
    green_leds = [cnt for cnt in contours_green if cv2.contourArea(cnt) > 100]
    red_leds = [cnt for cnt in contours_red if cv2.contourArea(cnt) > 100]


    if VIDEO:
        #contours
        frame_with_contours = hsv_frame.copy()
        cv2.drawContours(frame_with_contours, green_leds, -1, (0, 0, 255), 2)
        cv2.drawContours(frame_with_contours, red_leds, -1, (255, 0, 0), 2)
        #bright spots
        blurred = cv2.GaussianBlur(gray_frame, (11,11), 0)
        _, thresh = cv2.threshold(gray_frame, 200, 255, cv2.THRESH_BINARY)\
        
        countours_gray, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bright_spots = []
        for countour in countours_gray:
            M = cv2.moments(countour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                bright_spots.append((cX, cY))
        output = gray_frame.copy()
        for spots in bright_spots:
            cv2.circle(output, spots, 10, (0,255,0), -1)

        ####
        # check kleur om brightspot heen?
        #####
        # Look for green and red color around each bright spot
        # leds = {'green': [], 'red': []}
        # for spot in bright_spots:
        #     # Extract a region of interest (ROI) around the bright spot
        #     x, y = spot
        #     roi = hsv_frame[y - 10:y + 10, x - 10:x + 10]

        #detected_frame = cv2.bitwise_and(frame, frame, mask=mask)
    else:
        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, green_leds, -1, (0, 255, 0), 2)
        cv2.drawContours(image_with_contours, red_leds, -1, (0, 0, 255), 2)

        blurred = cv2.GaussianBlur(gray_image, (11,11), 0)
        _, thresh = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)\
        
        countours_gray, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bright_spots = []
        for countour in countours_gray:
            M = cv2.moments(countour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                bright_spots.append((cX, cY))
        output = gray_image.copy()
        for spots in bright_spots:
            cv2.circle(output, spots, 10, (0,255,0), -1)

        #detected_frame = cv2.bitwise_and(image, image, mask=mask)

    #cv2.imshow('led detection', detected_frame)
    if VIDEO:
        cv2.imshow('contours', frame_with_contours)
        cv2.imshow('gray', output)
    else:
        cv2.imshow('contours', image_with_contours)
        cv2.imshow('bright', output)
        #cv2.imshow('gray', gray_image)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()