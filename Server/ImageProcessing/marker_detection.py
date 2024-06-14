import cv2
from cv2 import aruco
import numpy as np
import time

try:
    # Try relative import if running as part of the package
    from .camera import Camera
except ImportError:
    # Fall back to direct import if running the script directly
    from camera import Camera
"""
to do:
- calibrate for potential distortion (not really needed for webcam, posibly needed for phone)
- enable both simulation and camera?
"""

class MarkerDetector:
    def __init__(self, camType1=0, cameraSource1=0, enableCam2=True, camType2=0, cameraSource2=0, debug=False):
        self.DEBUG = debug
        self.offset = 50
        self.enableCam2 = enableCam2
        self.cam1 = Camera(camType1, cameraSource1)
        if self.enableCam2:
            self.cam2= Camera(camType2, cameraSource2)
        

        # Setup marker detection
        self.detectorParams = aruco.DetectorParameters()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.detector = aruco.ArucoDetector(self.dictionary, self.detectorParams)

        # If debug draw windows
        if self.DEBUG:
            cv2.namedWindow("Output", cv2.WINDOW_AUTOSIZE)
            #cv2.namedWindow("Rejected", cv2.WINDOW_AUTOSIZE)
            if self.enableCam2:
                cv2.namedWindow("Output2", cv2.WINDOW_AUTOSIZE)
            #     cv2.namedWindow("Rejected2", cv2.WINDOW_AUTOSIZE)


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
        markers = {}
        input1 = self.cam1.getImage()
        if self.enableCam2:
            input2 = self.cam2.getImage()

        markersCorners1, markersId1, rejectedCandidates1 = self.detector.detectMarkers(input1)
        # Add detected markers from the first camera to the dictionary
        if markersId1 is not None:
            for id1, corner1 in zip(markersId1.flatten(), markersCorners1):
                markers[int(id1)] = corner1

        if self.enableCam2:
            markersCorners2, markersId2, rejectedCandidates2 = self.detector.detectMarkers(input2)
            if markersId2 is not None and markersCorners2 is not None:
                for corners, id in zip(markersCorners2, markersId2.flatten()):
                    if int(id) not in markers:
                        markers[int(id)] = corners

        # Convert combined results to the appropriate format
        markersCorners = list(markers.values())
        markersId = list(markers.keys())

        markersCorners = np.array(markersCorners, dtype=object)
        markersId = np.array(markersId).reshape(-1, 1)

        newMarkerInfoList = []
        # Process the combined markers
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
                if angle < 0:
                    angle += 360
                adjusted_centerX = int(centerX + self.offset * np.cos(np.radians(angle-90)))
                adjusted_centerY = int(centerY + self.offset * np.sin(np.radians(angle-90)))
                newMarkerInfoList.append({
                    'id': int(markerId),
                    'position': (adjusted_centerX, adjusted_centerY),
                    'angle': float(angle)
                })

            if self.DEBUG:
                # Draw an arrow indicating the orientation of the marker
                arrow_length = 50  # Length of the arrow
                for marker in newMarkerInfoList:
                    posX, posY = marker['position']
                    angle = marker['angle']
                    endX = int(posX + arrow_length * np.cos(np.radians(angle-90)))
                    endY = int(posY + arrow_length * np.sin(np.radians(angle-90)))
                    cv2.arrowedLine(input1, (posX, posY), (endX, endY), (255, 255, 0), 2, tipLength=0.3)
                corners_list = [np.array(corners, dtype=np.int32) for corners in markersCorners]
                aruco.drawDetectedMarkers(input1, corners_list, markersId)
                cv2.imshow("Output", input1)
                if self.enableCam2:
                #     corners_list2 = [np.array(corners, dtype=np.int32) for corners in markersCorners2]
                #     aruco.drawDetectedMarkers(input2, corners_list2, markersId2)
                    cv2.imshow("Output2", input2)

        if self.DEBUG:
            #rejected = input1.copy()
            #aruco.drawDetectedMarkers(rejected, rejectedCandidates1, borderColor=(100, 0, 255))
            #cv2.imshow("Rejected", rejected)
            # if self.enableCam2:
            #     rejected2 = input2.copy()
            #     aruco.drawDetectedMarkers(rejected2, rejectedCandidates2, borderColor=(100, 0, 255))
            #     cv2.imshow("Rejected2", rejected2)
            cv2.waitKey(1)  # Ensure the images are updated

        return newMarkerInfoList

    def releaseResources(self):
        self.cam1.release()
        if self.enableCam2:
            self.cam2.release()
        cv2.destroyAllWindows()

# Example usage:
if __name__ == "__main__":
    detector  = MarkerDetector(cameraSource1=0, camType1=0, enableCam2=False, cameraSource2=0, camType2=0, debug=True)
    #time.sleep(4)
    # Initialize variables for FPS calculation
    frame_count = 0
    start_time = time.time()
    while(True):
        beginTime = time.time_ns()
        markerInfoList = detector.detectMarkers()
        for mark in markerInfoList:
                print(f"id: {mark['id']}, pos: {mark['position']}, angle: {mark['angle']}")
        #time.sleep(2)
        endTime = (time.time_ns() -  beginTime) / 1000000
        # Increment frame count
        frame_count += 1
        #time.sleep(0.03333)

        # Calculate elapsed time
        elapsed_time = time.time() - start_time
        # Calculate FPS
        if elapsed_time > 1:  # Update FPS every second
            fps = frame_count / elapsed_time
            print(f"FPS: {fps:.2f}")
            
            # Reset frame count and start time for the next calculation
            frame_count = 0
            start_time = time.time()
 
    detector.releaseResources()
