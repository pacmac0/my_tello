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
        self.should_stop = False
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
        except:
            raise Exception("Could not start video stream")

    def run(self):
        self.update()
        if frame_read.stopped:
                frame_read.stop()
                return
        
        for event in self.user_io.event.get():
            if event.type == self.user_io.QUIT:
                    self.should_stop = True
            elif event.type == self.user_io.KEYDOWN:
                    if event.key == self.user_io.K_ESCAPE:
                        self.should_stop = True
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
                    self.should_stop = True
                elif event.type == self.user_io.KEYDOWN:
                    if event.key == self.user_io.K_ESCAPE:
                        self.should_stop = True
                    elif event.key == self.user_io.K_f: # toggle autonomous mode
                        self.autonomous_mode = not self.autonomous_mode
                        print("Autonomous flight mode deactivated!")
                
                

            else:
                if event.type == self.user_io.USEREVENT + 1:
                    self.update()
                elif event.type == self.user_io.QUIT:
                    self.should_stop = True
                elif event.type == self.user_io.KEYDOWN:
                    if event.key == self.user_io.K_ESCAPE:
                        self.should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == self.user_io.KEYUP:
                    self.keyup(event.key)
        return self.should_stop

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
        """ Update routine. Send velocities to Tello. """
        if self.send_rc_control:
            self.drone.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)

    def battery(self):
        return self.drone.get_battery()[:2]