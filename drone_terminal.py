from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import drone_controller

class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 50)

    def run(self):
        # init movement controller with drone device
        try:
            droneController = drone_controller.droneController()
        except:
            raise Exception("Couldn't initialize drone controller")
        
        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                        droneController.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        droneController.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    droneController.keyup(event.key)

            if droneController.frame_read.stopped:
                droneController.frame_read.stop()
                print("Frameread stopped land safely!")
            
            frame = pygame.surfarray.make_surface(droneController.get_frame()) # create pygame screen
            self.screen.blit(frame, (0, 0)) # put screen on
            pygame.display.update()

            time.sleep(1 / droneController.FPS)

        # Call it always before finishing. To deallocate resources.
        droneController.drone.end()

def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()