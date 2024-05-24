import cv2
from cv2 import aruco

markerImage = None
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

markerId = 0
sidePixels= 200
markerImage = aruco.generateImageMarker(dictionary, markerId, sidePixels, markerImage, 1)
cv2.imwrite("marker0.png", markerImage)

