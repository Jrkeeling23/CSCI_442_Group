import cv2 as cv
import numpy as np
import imageManipulation
import makeMoves
import robot_control
# import client
import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
import socket
import time
import queue
from facedetection import FaceDetection
from bin_ice_detection import Goal

"""
TODO: 
Find ranges for ice/goal colors.
Blob detection
Orientation
Talking when it needs to
Change state of robot in correct spots
"""


class Frame:
    """
    Frame class creates camera and creates grabs frames. It is used to use frames and manipulate frames for robot.
    """

    def __init__(self):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/V$
        self.camera = PiCamera()
        # camera.resolution = (640, 480)

        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.manipulation = imageManipulation.ImageManipulation()
        self.width = 640
        self.height = 480
        self.move = makeMoves.Move(self.width, self.height)

        self.status = False
        self.robot = Robot(goal="pink")

    def run(self):
        """
        A function to get frames and run the program.
        :return:
        """
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            self.rawCapture.truncate(0)
            img = frame.array

            if self.robot.finished is True:  # End Program!
                break

            if self.robot.start and self.robot.deliver:
                if self.robot.goal.bin_area(img) is False and self.robot.found_bin:  # Bin is out of view, but is found
                    self.robot.move.wheels_forward()  # get a little closer, if need be....
                    self.robot.move.drop()  # drop into box

                else:  # If it has not found the bin or it is still in view keep calling function to move robot
                    self.detect_bin(img)
                    self.robot.finished = True  # terminate program

            self.rawCapture.truncate(0)
            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()

    def detect_bin(self, frame):
        """
        This function detects the correct bins by blob detection.
        :param frame: current frame.
        :return:
        """
        if self.robot.goal.bin_area(frame) is False:  # turn 90 degrees
            self.robot.move.turn_right_90()

            if self.robot.goal.bin_area(frame) is False:  # Turn back 180 degrees
                self.robot.move.turn_left_90()
                self.robot.move.turn_left_90()
        else:
            self.robot.found_bin = True
            if self.robot.goal.current_x >= (self.width / 2 + self.width / 3 - 15):
                self.robot.move.turn_left()
            elif self.robot.goal.current_x <= (self.width / 3 + 15):
                self.robot.move.turn_right()
            else:
                self.robot.move.wheels_forward()


class Robot:
    """
    Robot class is used to control robot movements and commands. It uses Frame class heavily.
    """

    def __init__(self, goal):
        # variable to determine is robot has completed its task
        self.finished = False

        # variables to track robots location
        self.start = True
        self.mining_area = False
        self.rock_field = False

        # variables to track robots actions
        self.mine = False
        self.deliver = True

        self.face = FaceDetection()
        self.move = robot_control.MoveRobot()

        self.found_bin = False

        self.goal = Goal(goal)  # variable to track robots goal. String that is either Pink, Green, or Orange.


driver = Frame()
driver.run()
