import cv2

def find_camera_sources():
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            print(f"No camera found at index {i}")
        else:
            print(f"Camera found at index {i}")
            cap.release()

#find_camera_sources()
cap = cv2.VideoCapture(0)
while(True):


    ret, frame = cap.read()
    cv2.imshow('window', frame)