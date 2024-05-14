import cv2
import numpy as np
cap = cv2.VideoCapture('http://145.24.238.208:8080/video')


while(True):
    ret, frame = cap.read()

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #red
    #H 170=180
    #s 175 255
    #v 20 255

    #green
    #H 40 70
    #S 150 255
    #v 20 255
    lower_hsv_red = np.array([0, 175, 20])
    higher_hsv_red = np.array([10,255,255])

    lower_hsv_white = np.array([0, 0, 168])
    higher_hsv_white = np.array([172,111,255])

    lower_hsv_green = np.array([40, 150, 20])
    higher_hsv_green = np.array([70,255,255])

    mask_white = cv2.inRange(hsv_frame, lower_hsv_white,higher_hsv_white)
    #mask_red = cv2.inRange(hsv_frame, lower_hsv_red,higher_hsv_red)
    #mask_green = cv2.inRange(hsv_frame, lower_hsv_green, higher_hsv_green)
    mask = mask_white #mask_green + mask_red
    detected_frame = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('green', detected_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()