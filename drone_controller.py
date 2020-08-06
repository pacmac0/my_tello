from djitellopy import Tello
import pygame
import cv2
import numpy as np
import concurrent.futures
import time

"""
Command list:
w = up
a = left
s = down
d = right
f = toggle auto mode
l = land
t = take off
1 = speed lvl. 1 | distance lvl.1
2 = speed lvl. 2 | distance lvl.2
3 = speed lvl. 3 | distance lvl.3
4 = speed lvl. 4 | distance lvl.4
5 = speed lvl. 5 | distance lvl.5
6 = speed lvl. 6 | distance lvl.6
7 = speed lvl. 7 | distance lvl.7
8 = speed lvl. 8 | distance lvl.8
9 = speed lvl. 9 | distance lvl.9
o = reduce speed by 5
p = inhance speed by 5
g = lock oncurrent detection
"""

class droneController:
    def __init__(self):
        # Init Tello object that interacts with the Tello drone
        self.drone = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10 # initial speed
        self.S = 40 # speed of the drone when moving

        # drone controle toggles
        self.interrupt = False
        self.autonomous_mode = False
        self.send_rc_control = False

        # Frames per second of the pygame window display
        self.FPS = 60

        # autonomous properties
        # openCV detection box sizes (approximated to use as distance measure)
        # self.box_sizes = [1026, 684, 456, 304, 202, 136, 90]
        self.box_sizes = [680, 460, 300, 200, 140]
        self.distance_level = 2
        self.frame_size = {'x':960, 'y':720}
        self.frame_center = {'x':self.frame_size['x']/2, 'y':self.frame_size['y']/2}
        self.heightOffset = 150 # to let drone hover not directly infront of face
        self.tolerance = 40
        self.target_coordinates = {'x': self.frame_center['x'], 'y': self.frame_center['y'], 'z':400}
        self.face_lost = False

        try:
            self.drone.connect()
        except:
            raise Exception("Tello not connected")
        try:
            self.drone.set_speed(self.speed)
        except:
            raise Exception("setting speed not possible")
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

    
    def keydown(self, key):
        """ Update velocities based on key pressed (pressed key comands)
        Arguments:
            key: pygame key
        """
        if not self.autonomous_mode:
            if key == pygame.K_UP:  # set forward velocity
                self.for_back_velocity = self.S
            elif key == pygame.K_DOWN:  # set backward velocity
                self.for_back_velocity = -self.S
            elif key == pygame.K_LEFT:  # set left velocity
                self.left_right_velocity = -self.S
            elif key == pygame.K_RIGHT:  # set right velocity
                self.left_right_velocity = self.S
            elif key == pygame.K_w:  # set up velocity
                self.up_down_velocity = self.S
            elif key == pygame.K_s:  # set down velocity
                self.up_down_velocity = -self.S
            elif key == pygame.K_a:  # set yaw counter clockwise velocity
                self.yaw_velocity = -self.S
            elif key == pygame.K_d:  # set yaw clockwise velocity
                self.yaw_velocity = self.S
    
    def keyup(self, key):
        """ Update velocities based on key released (coand on key release)
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.drone.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.reset_velocities()
            self.drone.land()
            self.send_rc_control = False
        elif key == pygame.K_o: # speed up
                self.S += 5
        elif key == pygame.K_p: # speed down
                self.S -= 5
        elif key == pygame.K_f: # toggle autonomous mode
            self.autonomous_mode = not self.autonomous_mode
            if self.autonomous_mode:
                print("Autonomous flight mode active!")
                self.S = 20
                self.auto_flight()
            else:
                print("Autonomous flight mode deactived!")
                self.S = 40
            self.reset_velocities()
        
        if self.autonomous_mode:
            if key == pygame.K_1:
                self.distance_level = 0
            elif key == pygame.K_2:
                self.distance_level = 1
            elif key == pygame.K_3:
                self.distance_level = 2
            elif key == pygame.K_4:
                self.distance_level = 3
            elif key == pygame.K_5:
                self.distance_level = 4
        else:
            if key == pygame.K_1:
                self.S = 10
            elif key == pygame.K_2:
                self.S = 20
            elif key == pygame.K_3:
                self.S = 30
            elif key == pygame.K_4:
                self.S = 40
            elif key == pygame.K_5:
                self.S = 50
            elif key == pygame.K_6:
                self.S = 60
            elif key == pygame.K_7:
                self.S = 70
            elif key == pygame.K_8:
                self.S = 80
            elif key == pygame.K_9:
                self.S = 90
            elif key == pygame.K_0:
                self.S = 100

        return self.autonomous_mode

    def auto_flight(self):
        """
        drone follows autonomously detected face or object in defined distance
        """
        # center face in frame
        frame_center_vector = np.array((self.frame_center['x'], self.frame_center['y'], self.box_sizes[self.distance_level]))
        target_position_vector = np.array((self.target_coordinates['x'], self.target_coordinates['y'] + self.heightOffset, self.target_coordinates['z']))
        distance_vector = frame_center_vector - target_position_vector
        # print("Distance Vector ({}, {}, {})".format(distance_vector[0],distance_vector[1],distance_vector[2]))
        
        if self.face_lost:
            self.face_search_hover()
        else:
            # rotate (r-axes)
            if distance_vector[0] < -self.tolerance:
                self.yaw_velocity = int(self.S)
            elif distance_vector[0] > self.tolerance:
                self.yaw_velocity = int(-self.S)
            else:
                self.yaw_velocity = 0
            
            # up down (y-axes)
            if distance_vector[1] < -self.tolerance:
                self.up_down_velocity = -self.S
            elif distance_vector[1] > self.tolerance:
                self.up_down_velocity = self.S
            else:
                self.up_down_velocity = 0
            
            # back forth (z-axes)
            if distance_vector[2] > 0:
                self.for_back_velocity = self.S
            elif distance_vector[2] < 0:
                self.for_back_velocity = -self.S
            else:
                self.for_back_velocity = 0
        
                
    def reset_velocities(self):
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.drone.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)

    def get_frame(self):
        """
        puts together the frame processing pipeline and returns the frame
        """
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
            self.face_lost = False
            for (x,y,w,h) in faces:
                self.target_coordinates['x'] = int(x + w/2)
                self.target_coordinates['y'] = int(y + h/2)
                self.target_coordinates['z'] = w*2
                cv2.rectangle(self.frame, (x, y), (x+w, y+h),(255, 0, 255), 2)
                # print("face detected with size: ({} ,{})".format(str(x),str(y)))
                # draw targeting cicle if face detected
                cv2.circle(self.frame, (int(self.target_coordinates['x']), int(self.target_coordinates['y'])), 10, (0,255,0), 2)
        else:
            if not self.face_lost:
                self.cycle_start = time.time()
            self.face_lost = True
            self.target_coordinates['x'] = self.frame_center['x']
            self.target_coordinates['y'] = self.frame_center['y']
            self.target_coordinates['z'] = self.box_sizes[3]
        # draw frame center point
        cv2.circle(self.frame, (int(self.frame_center['x']), int(self.frame_center['y'])), 2, (0,0,255), 2)
        # in autonomus mode draw a target line from frame_center(view point) to target cicle
        if self.autonomous_mode:
            cv2.line(self.frame, (int(self.frame_center['x']), int(self.frame_center['y'])), (int(self.target_coordinates['x']), int(self.target_coordinates['y'])), [255, 0, 0], 1)
        # show stats
        cv2.putText(self.frame,"for/back velocity: {}".format(self.for_back_velocity),(5,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        cv2.putText(self.frame,"up/down velocity: {}".format(self.up_down_velocity),(5,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        cv2.putText(self.frame,"left/right velocity: {}".format(self.left_right_velocity),(5,70),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        cv2.putText(self.frame,"Speed: {}".format(self.S),(5,90),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        cv2.putText(self.frame,"Auto: {}".format(self.autonomous_mode),(5,110),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        cv2.putText(self.frame,"Auto Dist: {}".format(self.distance_level+1),(5,130),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        
        try:
            battery_level = self.drone.get_battery()[:2]
            cv2.putText(self.frame,"Battery: {}".format(battery_level),(5,150),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)
        except:
            cv2.putText(self.frame,"Battery: unknown",(5,150),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2)

    def face_search_hover(self):
        # hover up and down to find the face again
        if self.cycle_start+2 > time.time():
            self.up_down_velocity = self.S
            self.for_back_velocity = -self.S
        else:
            self.up_down_velocity = -self.S
            self.for_back_velocity = self.S
            self.cycle_start = time.time()
        

    def clearImage(self):
        # image clearance
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB) # color converion
        self.frame = np.rot90(self.frame) # rotate image correct
        self.frame = np.flipud(self.frame)