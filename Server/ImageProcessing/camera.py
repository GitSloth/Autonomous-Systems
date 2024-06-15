import cv2, time
import numpy as np
from threading import Thread
import platform

class Camera:
    CAMERA = 0
    WEBCAM = 1 #not really used rn
    WEB = 2

    def __init__(self,camType,source=0):
        self.camType = camType
        self.threadWait = False
        if self.camType == 0:
            self.capture = cv2.VideoCapture(source)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.frame = np.zeros((1280,720, 3), dtype = np.uint8)
            self.FPS = 1/60
            self.FPS_MS = int(self.FPS * 1000)
            self.thread = Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()
        if self.camType == 1: #not used rn
            if platform.system() == 'Windows':
                self.capture = cv2.VideoCapture(source, cv2.CAP_DSHOW) #without dshow it takes 10 minutes for the camera to open.
            else:
                self.capture = cv2.VideoCapture(source)
            self.frame = np.zeros((1280,720, 3), dtype = np.uint8)
        if self.camType == 2:
            self.capture = cv2.VideoCapture(source)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.frame = np.zeros((1280,720, 3), dtype = np.uint8)
            #self.FPS = 1/60
            #self.FPS_MS = int(self.FPS * 1000)
            self.thread = Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()

        

    def update(self):
        self.index = 0
        #self.timeDinge = time.time
        while True:
            if self.threadWait:
                continue
            
            if self.capture.isOpened():
                (self.status, self.frame) =  self.capture.read()
                
            #if self.camType == 0:
            #   time.sleep(self.FPS)
 
    
    def show_frame(self):
        cv2.imshow('frame', self.frame)
        cv2.waitKey(self.FPS_MS)   

    def getImage(self):
        return self.frame
    
    def release(self):
        self.capture.release()