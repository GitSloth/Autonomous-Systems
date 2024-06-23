import cv2
from cv2 import aruco

'''
This file allows for the generation of ArUco markers using the opencv implementation
'''
markerImage = None
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)


sidePixels = 256
for i in range(20):
    markerImage = aruco.generateImageMarker(dictionary, i, sidePixels, markerImage, 1)
    cv2.imwrite(f"marker{i}.png", markerImage)

