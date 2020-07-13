from djitellopy import Tello
import pygame
import cv2
import numpy as np
import concurrent.futures

class droneController:
    def __init__(self, io):
        # Init Tello object that interacts with the Tello drone
        self.drone = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        # drone controle toggles
        self.interrupt = False
        self.autonomous_mode = False
        self.send_rc_control = False

        # Speed of the drone
        self.S = 60
        # Frames per second of the pygame window display
        self.FPS = 60
        self.user_io = io #pygame control to get input

        try:
            self.drone.connect()
        except:
            raise Exception("Tello not connected")
        try:
            self.drone.set_speed(self.speed)
        except:
            raise Exception("Not set speed to lowest possible")
        # In case streaming is on. This happens when we quit this program without the escape key.
        try:
            self.drone.streamoff()
        except:
            raise Exception("Could not stop video stream")
        try:
            self.drone.streamon()
            self.frame_read = self.drone.get_frame_read()
        except:
            raise Exception("Could not start video stream")
        # init video processing
        try:
            # Set up recognition module
            try:
                self.face_cascade = cv2.CascadeClassifier()
                self.face_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_frontalface_alt2.xml'))
            except:
                raise Exception("loading face cascade failed")
            """
            try:
                self.full_body_cascade = cv2.CascadeClassifier()
                self.full_body_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_fullbody.xml')):
            except:
                raise Exception("loading full body cascade failed")
            try:
                self.profile_face_cascade = cv2.CascadeClassifier()
                self.profile_face_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_profileface.xml')):
            except:
                raise Exception("loading face profile cascade failed")
            try:
                self.upper_body_cascade = cv2.CascadeClassifier()
                self.upper_body_cascade.load(cv2.samples.findFile('cascades/data/haarcascade_upperbody.xml')):
            except:
                raise Exception("loading upper body cascade failed")
            """
        except:
            raise Exception("Could not init recognition models")

    def run_controlles(self):
        self.update()

    def run(self):

        for event in self.user_io.event.get():
            if event.type == self.user_io.QUIT:
                    self.interrupt = True
            elif event.type == self.user_io.KEYDOWN:

                    if event.key == self.user_io.K_ESCAPE:
                        self.interrupt = True
                    elif event.key == self.user_io.K_f: # toggle autonomous mode
                        self.autonomous_mode = not self.autonomous_mode
                        if self.autonomous_mode:
                            print("Autonomous flight mode active!")
                        else:
                            print("Autonomous flight mode deactivated!")
            
        
            if self.autonomous_mode:
                if event.type == self.user_io.USEREVENT + 1:
                    self.update()
                elif event.type == self.user_io.QUIT:
                    self.interrupt = True
                elif event.type == self.user_io.KEYDOWN:
                    if event.key == self.user_io.K_ESCAPE:
                        self.interrupt = True
                    elif event.key == self.user_io.K_f: # toggle autonomous mode
                        self.autonomous_mode = not self.autonomous_mode
                        print("Autonomous flight mode deactivated!")
                
                

            else:
                if event.type == self.user_io.USEREVENT + 1:
                    self.update()
                elif event.type == self.user_io.QUIT:
                    self.interrupt = True
                elif event.type == self.user_io.KEYDOWN:
                    if event.key == self.user_io.K_ESCAPE:
                        self.interrupt = True
                    else:
                        self.keydown(event.key)
                elif event.type == self.user_io.KEYUP:
                    self.keyup(event.key)
        return self.interrupt
=======
                if event.key == self.user_io.K_ESCAPE:
                    interrupt = True
                else:
                    self.keydown(event.key)
            elif event.type == self.user_io.KEYUP:
                self.keyup(event.key)
>>>>>>> parent of a648fff... speed analyses

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == self.user_io.K_UP:  # set forward velocity
            self.for_back_velocity = self.S
        elif key == self.user_io.K_DOWN:  # set backward velocity
            self.for_back_velocity = -self.S
        elif key == self.user_io.K_LEFT:  # set left velocity
            self.left_right_velocity = -self.S
        elif key == self.user_io.K_RIGHT:  # set right velocity
            self.left_right_velocity = self.S
        elif key == self.user_io.K_w:  # set up velocity
            self.up_down_velocity = self.S
        elif key == self.user_io.K_s:  # set down velocity
            self.up_down_velocity = -self.S
        elif key == self.user_io.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -self.S
        elif key == self.user_io.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = self.S
        elif key == self.user_io.K_b: # get battery level
            print("%s%% battery left" %self.battery())
        elif key == self.user_io.K_f: # toggle autonomous mode
            self.autonomous_mode = not self.autonomous_mode
            print("Autonomous flight mode active!")

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == self.user_io.K_UP or key == self.user_io.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == self.user_io.K_LEFT or key == self.user_io.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == self.user_io.K_w or key == self.user_io.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == self.user_io.K_a or key == self.user_io.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == self.user_io.K_t:  # takeoff
            self.drone.takeoff()
            self.send_rc_control = True
        elif key == self.user_io.K_l:  # land
            self.drone.land()
            self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.drone.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)

    def run_video(self):
        """
        puts together the frame processing pipeline and returns the frame
        """
        if self.frame_read.stopped:
                self.frame_read.stop()
                self.interrupt = True
                
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

    def battery(self):
        return self.drone.get_battery()[:2]