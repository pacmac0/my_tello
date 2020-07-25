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
        self.S = 20 # speed of the drone when moving

        # drone controle toggles
        self.interrupt = False
        self.autonomous_mode = False
        self.send_rc_control = False

        # Frames per second of the pygame window display
        self.FPS = 60

        # autonomous properties
        # openCV detection box sizes (approximated to use as distance measure)
        # self.box_sizes = [1026, 684, 456, 304, 202, 136, 90]
        self.box_sizes = [136, 202, 304, 456, 684]
        self.distance_level = 1
        self.frame_size = {'x':960, 'y':720}
        self.frame_center = {'x':self.frame_size['x']/2, 'y':self.frame_size['y']/2}
        self.heightOffset = 150
        self.tolerance = 40
        self.target_coordinates = {'x': self.frame_center['x'], 'y': self.frame_center['y'], 'z':400}
        # restrict detection mechanism to every x seconds to stabalize auto flight and speed up processing
        self.wait_time = 1.5
        self.last_detect = time.time()
        self.locked = False
        self.lock_on_offset = 300

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
        elif key == pygame.K_b: # get battery level
            print("{} percent battery left".format(self.battery()))
        elif key == pygame.K_f: # toggle autonomous mode
            self.autonomous_mode = not self.autonomous_mode
            if self.autonomous_mode:
                print("Autonomous flight mode active!")
                self.reset_velocities()
            else:
                print("Autonomous flight mode deactived!")
                self.reset_velocities()
                self.auto_flight()
        
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
            elif key == pygame.K_g:
                self.lock_on()
        else:
            if key == pygame.K_1:
                self.S = 20
            elif key == pygame.K_2:
                self.S = 40
            elif key == pygame.K_3:
                self.S = 60
            elif key == pygame.K_4:
                self.S = 80
            elif key == pygame.K_5:
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
        
        # rotate (r-axes)
        if distance_vector[0] < -self.tolerance:
            self.yaw_velocity = int(self.S/2)
        elif distance_vector[0] > self.tolerance:
            self.yaw_velocity = int(-self.S/2)
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
        if (self.last_detect - time.time()) < self.wait_time:
            self.detect()
            self.last_detect = time.time()
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
                if self.locked:
                    if (not (self.target_coordinates['x']-self.lock_on_offset <= int(x + w/2)) or 
                        not (int(x + w/2) <= self.target_coordinates['x']+self.lock_on_offset) or 
                        not (self.target_coordinates['y']-self.lock_on_offset <= int(y + h/2)) or 
                        not (int(y + h/2) <= self.target_coordinates['y']+self.lock_on_offset)):
                        continue
                    else:
                        self.target_coordinates['x'] = int(x + w/2)
                        self.target_coordinates['y'] = int(y + h/2)
                        self.target_coordinates['z'] = w*2
                        cv2.rectangle(self.frame, (x, y), (x+w, y+h),(255, 0, 255), 2)
                        # print("face detected with size: ({} ,{})".format(str(x),str(y)))
                        # draw targeting cicle if face detected
                        cv2.circle(self.frame, (int(self.target_coordinates['x']), int(self.target_coordinates['y'])), 10, (0,255,0), 2)    
                else:
                    self.target_coordinates['x'] = int(x + w/2)
                    self.target_coordinates['y'] = int(y + h/2)
                    self.target_coordinates['z'] = w*2
                    cv2.rectangle(self.frame, (x, y), (x+w, y+h),(255, 0, 255), 2)
                    # print("face detected with size: ({} ,{})".format(str(x),str(y)))
                    # draw targeting cicle if face detected
                    cv2.circle(self.frame, (int(self.target_coordinates['x']), int(self.target_coordinates['y'])), 10, (0,255,0), 2)
        else:
            self.target_coordinates['x'] = self.frame_center['x']
            self.target_coordinates['y'] = self.frame_center['y']
            self.target_coordinates['z'] = self.box_sizes[3]
        # draw frame center point
        cv2.circle(self.frame, (int(self.frame_center['x']), int(self.frame_center['y'])), 2, (0,0,255), 2)

    def lock_on(self):
        if self.locked:
            self.locked_coordinates = self.target_coordinates
            self.locked = not self.locked
        else:
            self.locked_coordinates = self.target_coordinates
            self.locked = not self.locked

    def clearImage(self):
        # image clearance
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB) # color converion
        self.frame = np.rot90(self.frame) # rotate image correct
        self.frame = np.flipud(self.frame)

    def battery(self):
        return self.drone.get_battery()[:2]