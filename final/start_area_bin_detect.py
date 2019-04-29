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
                # TODO: edge detect white. Find if highest white is left or right. Turn towards highest white...
                white_lower = np.array([250, 250, 250])
                white_upper = np.array([255, 255, 255])

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
        white_lower = np.array([250, 250, 250])
        white_upper = np.array([255, 255, 255])
        white_range = cv.inRange(frame, white_lower, white_upper)

        if self.robot.goal.detect_bin(frame) is False:  # Must turn to find bin...
            # TODO: Turn 90 degrees right!
            # TODO: Turn 180 degrees left!
            # todo: This allows a full span



        self.robot.move.arm_in_cam_view()  # get arm into position
        if self.robot.goal.detect_ice(frame) is True:
            time.sleep(1)  # wait 5 seconds
            self.robot.move.close_hand()
            # change states of robot to fit accordingly
            self.robot.mine = False
            self.robot.deliver = True
            time.sleep(4)  # wait 5 seconds
            self.robot.move.turn_around()
            time.sleep(1)  # wait 5 seconds
            self.robot.move.lower_arm()
        else:
            waste = None
            # TODO: Rejects ice with talk


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

        self.goal = Goal(goal)  # variable to track robots goal. String that is either Pink, Green, or Orange.


driver = Frame()
driver.run()
