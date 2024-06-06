import cv2
from cv2 import aruco
from camera import Camera
import numpy as np
import time
"""
to do:
- calibrate for potential distortion (not really needed for webcam, posibly needed for phone)
- enable both simulation and camera?
"""

class MarkerDetector2:
    def __init__(self, camType1=0, cameraSource1=0, enableCam2=False, camType2=0, cameraSource2=0, debug=False):
        self.DEBUG = debug
        self.enableCam2 = enableCam2
        self.cam1 = Camera(camType1, cameraSource1)
        if self.enableCam2:
            self.cam2= Camera(camType2, cameraSource2)
        self.markerInfoList = []

        # Setup marker detection
        self.detectorParams = aruco.DetectorParameters()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.detector = aruco.ArucoDetector(self.dictionary, self.detectorParams)

        # If debug draw windows
        if self.DEBUG:
            cv2.namedWindow("Output", cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow("Rejected", cv2.WINDOW_AUTOSIZE)

    def calculateAngle(self, corner1, corner2):
        """
        Calculate the angle between the vector formed by two corners and the vertical axis.
        :param corner1: First corner point (x, y).
        :param corner2: Second corner point (x, y).
        :return: Angle in degrees.
        """
        vector = corner2 - corner1
        angleRad = np.arctan2(vector[1], vector[0])  # Calculate angle in radians
        angleDeg = np.degrees(angleRad)  # Convert to degrees
        return angleDeg

    def detectMarkers(self):
        """
        Get the position from the markers using image from cam.
        :return: returns markerInfo
        """
        input1 = self.cam1.getImage()
        if self.enableCam2:
            input2 = self.cam2.getImage()

         
        markersCorners, markersId, rejectedCandidates = self.detector.detectMarkers(input1)
        if self.enableCam2:
            markersCorners2, markersId2, rejectedCandidates2 = self.detector.detectMarkers(input2)
            
        #aruco.drawDetectedMarkers(input1, markersCorners, markersId)
        
        if markersId is not None:
            markers = sorted(zip(markersId, markersCorners), key=lambda x: x[0])
            for markerId, corners in markers:
                # Calculate the center of the marker
                corners = corners.reshape((4, 2))
                centerX = int(corners[:, 0].mean())
                centerY = int(corners[:, 1].mean())
                if self.DEBUG:
                    cv2.circle(input1, (centerX, centerY), 5, (0, 255, 0), -1)
                # Calculate the angle of the marker
                topLeft = corners[0]
                topRight = corners[1]
                angle = self.calculateAngle(topLeft, topRight)
                # Check if the marker ID is already in the list
                markerFound = False
                for markerInfo in self.markerInfoList:
                    if markerInfo['id'] == markerId:
                        # Update the position and angle
                        markerInfo['position'] = (centerX, centerY)
                        markerInfo['angle'] = angle
                        markerFound = True
                        break
                if not markerFound:
                    # Add new marker information to the list
                    self.markerInfoList.append({
                        'id': markerId,
                        'position': (centerX, centerY),
                        'angle': angle
                    })

            if self.DEBUG:
                # Draw an arrow indicating the orientation of the marker
                arrow_length = 50  # Length of the arrow
                endX = int(centerX + arrow_length * np.cos(np.radians(angle)))
                endY = int(centerY + arrow_length * np.sin(np.radians(angle)))
                cv2.arrowedLine(input1, (centerX, centerY), (endX, endY), (255, 255, 0), 2, tipLength=0.3)
                #rejected = input.copy()
                aruco.drawDetectedMarkers(input1, markersCorners, markersId)
                #aruco.drawDetectedMarkers(rejected, rejectedCandidates, borderColor=(100, 0, 255))
        cv2.imshow("Output", input1)
        #cv2.imshow("Rejected", rejected)
        cv2.waitKey(1)  # Ensure the images are updated
        return self.markerInfoList

    def releaseResources(self):
        self.cam1.release()
        cv2.destroyAllWindows()

# Example usage:
if __name__ == "__main__":
    detector  = MarkerDetector2(cameraSource1='http://145.24.243.133:5005/video_feed', camType1=2, cameraSource2='http://145.24.243.132:8080/video', camType2=2, debug=True)
    
    while(True):
        beginTime = time.time_ns()
        markerInfoList = detector.detectMarkers()
        endTime = (time.time_ns() -  beginTime) / 1000000
        
        print(f"time: {endTime}")
        
        #time.sleep(.5)

        #print(f"endtime: {endTime}")
        #if markerInfoList is not None:
        #    for markerInfo in markerInfoList:
        #        print(f"Marker ID: {markerInfo['id']}, Position: {markerInfo['position']}, Angle: {markerInfo['angle']:.2f} degrees")
    detector.releaseResources()

# DEBUG = False

# def calculate_angle(corner1, corner2):
#     """
#     Calculate the angle between the vector formed by two corners and the vertical axis.
#     :param corner1: First corner point (x, y).
#     :param corner2: Second corner point (x, y).
#     :return: Angle in degrees.
#     """
#     vector = corner2 - corner1
#     angle_rad = np.arctan2(vector[1], vector[0])  # Calculate angle in radians
#     angle_deg = np.degrees(angle_rad)  # Convert to degrees
#     return angle_deg

# print("setup camera")
# cam = Camera(Camera.CAMERA, source=0)
# print("setup marker detection")
# markersId = []
# markersCorners = []
# rejectedCandidates = []
# marker_info_list = []

# detectorParams = aruco.DetectorParameters()
# dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
# detector = aruco.ArucoDetector(dictionary, detectorParams)
# previousTime = time.time_ns()
# interval = 200000000 # 0.2 seconds
# # List to store marker information


# print("setup window")
# cv2.namedWindow("output", cv2.WINDOW_AUTOSIZE) 
# if DEBUG:
#     cv2.namedWindow("rejected", cv2.WINDOW_AUTOSIZE)

# print("Setup complete")
# while True:
#     input = cam.getImage()
#     markersCorners, markersId, rejectedCandidates = detector.detectMarkers(input)
    
#     output = input.copy()
#     aruco.drawDetectedMarkers(output, markersCorners, markersId)
#     if markersId is not None:
        
#         markers = sorted(zip(markersId, markersCorners), key=lambda x: x[0])
#         for marker_id, corners in markers:
#             # Calculate the center of the marker
#             corners = corners.reshape((4, 2))
#             center_x = int(corners[:, 0].mean())
#             center_y = int(corners[:, 1].mean())
#             # Optionally, draw the center point on the output image
#             cv2.circle(output, (center_x, center_y), 5, (0, 255, 0), -1)
#              # Calculate the angle of the marker
#             top_left = corners[0]
#             top_right = corners[1]
#             angle = calculate_angle(top_left, top_right)
#             # Check if the marker ID is already in the list
#             marker_found = False
#             for marker_info in marker_info_list:
#                 if marker_info['id'] == marker_id:
#                     # Update the position and angle
#                     marker_info['position'] = (center_x, center_y)
#                     marker_info['angle'] = angle
#                     marker_found = True
#                     break
#             if not marker_found:
#                 # Add new marker information to the list
#                 marker_info_list.append({
#                     'id': marker_id,
#                     'position': (center_x, center_y),
#                     'angle': angle
#                 })
            
#             #print(f"Marker ID: {marker_id}, Position: ({center_x}, {center_y}), Angle: {angle:.2f} degrees")
#             # Draw an arrow indicating the orientation of the marker
#             arrow_length = 50  # Length of the arrow
#             end_x = int(center_x + arrow_length * np.cos(np.radians(angle)))
#             end_y = int(center_y + arrow_length * np.sin(np.radians(angle)))
#             cv2.arrowedLine(output, (center_x, center_y), (end_x, end_y), (255, 255, 0), 2, tipLength=0.3)
            
#             # Optionally, draw the center point and angle on the output image
#             cv2.circle(output, (center_x, center_y), 5, (0, 255, 0), -1)
#             #cv2.putText(output, f"{angle:.2f} deg", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#     currentTime = time.time_ns()    
#     if (currentTime - previousTime > interval):
#         print("==========================")
#         for info in marker_info_list:
#             print(f"Marker ID: {info['id']}, Position: {info['position']}, Angle: {info['angle']:.2f} degrees")
#         previousTime = currentTime
#     if DEBUG:
#         rejected = input.copy()
#         aruco.drawDetectedMarkers(rejected, rejectedCandidates, borderColor=(100, 0, 255))
#         cv2.imshow("rejected", rejected)
    
#     cv2.imshow("output", output)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# cam.release()
# cv2.destroyAllWindows()