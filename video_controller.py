import cv2
import numpy as np
import concurrent.futures

class videoController:
    def __init__(self, device):
        self.frame_read = device.get_frame_read()
        # Set up recognition module
        self.face_cascade = cv2.CascadeClassifier()
        #self.full_body_cascade = cv2.CascadeClassifier()
        #self.profile_face_cascade = cv2.CascadeClassifier()
        self.upper_body_cascade = cv2.CascadeClassifier()
        if not self.face_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_frontalface_alt2.xml')):
            print('--(!)Error loading face cascade')
            exit(-1)
        recognizer = cv2.face.LBPHFaceRecognizer_create()
        """
        if not self.full_body_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_fullbody.xml')):
            print('--(!)Error loading full body cascade')
            exit(-1)
        if not self.profile_face_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_profileface.xml')):
            print('--(!)Error loading profile face cascade')
            exit(-1)
        if not self.upper_body_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_upperbody.xml')):
            print('--(!)Error loading upper body cascade')
            exit(-1)
        """

    def get_frame(self):
        self.frame = self.frame_read.frame
        self.detect()
        self.clearImage()
        return self.frame

    def detect(self):
        """
        detection and video manipulation (drawing)
        """
        gray  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=2)
        """
        upper_bodys = self.upper_body_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=2)
        
        if len(upper_bodys) != 0:
            for (x,y,w,h) in upper_bodys:
                cv2.rectangle(self.frame, (x, y), (x+w, y+h),(0, 255, 0), 2)
        """
        if len(faces) != 0:
            for (x,y,w,h) in faces:
                center = (x + w//2, y + h//2)
                cv2.ellipse(self.frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255))
        


    def clearImage(self):
        # image clearance
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB) # color converion
        self.frame = np.rot90(self.frame) # rotate image correct
        self.frame = np.flipud(self.frame)