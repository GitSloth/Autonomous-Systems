import cv2

def find_camera_sources():
    '''
    helps finding the right camera source to use'''
    for i in range(10):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print(f"No camera found at index {i}")
        else:
            print(f"Camera found at index {i}")
            cap.release()

find_camera_sources()
