from controller import Robot, Camera
import cv2
import numpy as np
from cv2 import aruco

robot = Robot()

# 
timeStep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timeStep)


window_width = 800
window_height = 600

cv2.namedWindow("Camera Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera Image", window_width, window_height)

cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("ArUco Detection", window_width, window_height)


detectorParams = aruco.DetectorParameters()
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
detector = aruco.ArucoDetector(dictionary, detectorParams)

while robot.step(timeStep) != -1:

    image = camera.getImage()

    if image:
        width = camera.getWidth()
        height = camera.getHeight()    
            
        image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        bgr_image = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(bgr_image)

        if markerIds is not None:
            aruco.drawDetectedMarkers(bgr_image, markerCorners, markerIds)
            for markerId, corners in zip(markerIds, markerCorners):
                corners = corners.reshape((4, 2))
                centerX = int(corners[:, 0].mean())
                centerY = int(corners[:, 1].mean())
                cv2.circle(bgr_image, (centerX, centerY), 5, (0, 255, 0), -1)
                topLeft = corners[0]
                topRight = corners[1]
                angle = np.degrees(np.arctan2(topRight[1] - topLeft[1], topRight[0] - topLeft[0]))
                arrow_length = 50
                endX = int(centerX + arrow_length * np.cos(np.radians(angle)))
                endY = int(centerY + arrow_length * np.sin(np.radians(angle)))
                cv2.arrowedLine(bgr_image, (centerX, centerY), (endX, endY), (255, 255, 0), 2, tipLength=0.3)
                print(f"Marker ID: {markerId[0]}, Position: ({centerX}, {centerY}), Angle: {angle:.2f} degrees")
        
        cv2.imshow("Camera Image", bgr_image)
        cv2.imshow("ArUco Detection", bgr_image)
        

        cv2.waitKey(1)
        

# Cleanup
camera.disable()
cv2.destroyAllWindows()
