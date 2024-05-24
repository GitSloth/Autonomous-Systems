import cv2
from cv2 import aruco
from Camera import Camera
import numpy as np
def calculate_angle(corner1, corner2):
    """
    Calculate the angle between the vector formed by two corners and the vertical axis.
    :param corner1: First corner point (x, y).
    :param corner2: Second corner point (x, y).
    :return: Angle in degrees.
    """
    vector = corner2 - corner1
    angle_rad = np.arctan2(vector[1], vector[0])  # Calculate angle in radians
    angle_deg = np.degrees(angle_rad)  # Convert to degrees
    return angle_deg
print("setup camera")
cam = Camera(Camera.CAMERA, source=0)
print("setup marker detection")
markersId = []
markersCorners = []
rejectedCandidates = []

detectorParams = aruco.DetectorParameters()
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
detector = aruco.ArucoDetector(dictionary, detectorParams)

print("setup window")
cv2.namedWindow("output", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("rejected", cv2.WINDOW_AUTOSIZE)

while True:
    input = cam.getImage()
    markersCorners, markersId, rejectedCandidates = detector.detectMarkers(input)
    
    output = input.copy()
    aruco.drawDetectedMarkers(output, markersCorners, markersId)
    if markersId is not None:
        for marker_id, corners in zip(markersId, markersCorners):
            # Calculate the center of the marker
            corners = corners.reshape((4, 2))
            center_x = int(corners[:, 0].mean())
            center_y = int(corners[:, 1].mean())
            # Optionally, draw the center point on the output image
            cv2.circle(output, (center_x, center_y), 5, (0, 255, 0), -1)
             # Calculate the angle of the marker
            top_left = corners[0]
            top_right = corners[1]
            angle = calculate_angle(top_left, top_right)
            print(f"Marker ID: {marker_id}, Position: ({center_x}, {center_y}), Angle: {angle:.2f} degrees")
            # Draw an arrow indicating the orientation of the marker
            arrow_length = 50  # Length of the arrow
            end_x = int(center_x + arrow_length * np.cos(np.radians(angle-90)))
            end_y = int(center_y + arrow_length * np.sin(np.radians(angle-90)))
            cv2.arrowedLine(output, (center_x, center_y), (end_x, end_y), (255, 255, 0), 2, tipLength=0.3)
            
            # Optionally, draw the center point and angle on the output image
            cv2.circle(output, (center_x, center_y), 5, (0, 255, 0), -1)
            #cv2.putText(output, f"{angle:.2f} deg", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    rejected = input.copy()
    aruco.drawDetectedMarkers(rejected, rejectedCandidates, borderColor=(100, 0, 255))
    cv2.imshow("output", output)
    cv2.imshow("rejected", rejected)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break