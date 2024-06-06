import cv2
cv2.CAP_DSHOW
def find_camera_sources():
    for i in range(10):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print(f"No camera found at index {i}")
        else:
            print(f"Camera found at index {i}")
            cap.release()

find_camera_sources()
