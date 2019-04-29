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
        self.human_close = False
        self.robot = Robot(goal="l")

    def run(self):
        """
        A function to get frames and run the program.
        :return:
        """
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            img = frame.array

            if self.robot.finsihed is True:  # End Program!
                break

            if (self.robot.start or self.robot.rock_field) and self.robot.mine:  # move through rock field
                flood_fill_image = self.create_furthest_path()
                # TODO: robot movements based off of above image.

            if self.robot.mining_area and self.robot.mine:  # grab ice
                status = self.robot.face.detect_face(img)
                if status is True:
                    self.detect_ice()
                else:
                    continue  # Allows robot to keep calling face detection rather than

            if (self.robot.rock_field or self.robot.mining_area) and self.robot.deliver:  # must deliver
                if self.orientate() is False:
                    # TODO: spin until it finds a bin.
                    waste = None  # just to bypass error
                else:
                    flood_fill_image = self.create_furthest_path()
                    # TODO: robot movements based off of above image.

            if self.robot.start and self.robot.deliver:  # Find bin
                self.detect_bin()

            overlay = self.create_furthest_path(img.copy())  # create furthest non obstructed path
            cv.imshow("Overlay", overlay)

            self.rawCapture.truncate(0)
            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()

    def create_furthest_path(self, img):
        """
        This function detects a path in which the robot can travel without hitting obstacles.
        :param img: original frame that is to be manipulated
        :return: furthest path with original image
        """
        # values for boundary colors
        blue_lower = np.array([20, 20, 180])
        blue_upper = np.array([130, 60, 255])
        orange_lower = np.array([0, 50, 225])
        orange_upper = np.array([30, 255, 255])

        img = cv.blur(img, (5, 5))  # blur initial frame for edge detection
        hsv = cv.cvtColor(img.copy(), cv.COLOR_BGR2HSV)  # convert to HSV to do object detection

        blue_mask = cv.inRange(hsv, blue_lower, blue_upper)  # detect blue
        orange_mask = cv.inRange(hsv, orange_lower, orange_upper)  # detect orange

        # create orange and blue edges to subtract from overall picture
        kernel = np.ones((5, 5), np.uint8)
        blue_edge = self.manipulation.edge_detection(blue_mask)
        blue_edge = cv.dilate(blue_edge, kernel, iterations=1)
        orange_edge = self.manipulation.edge_detection(orange_mask)
        orange_edge = cv.dilate(orange_edge, kernel, iterations=1)

        # subtract the edges from the overall edge detection image
        image = self.manipulation.edge_detection(img.copy())
        image = cv.subtract(image, blue_edge)
        image = cv.subtract(image, orange_edge)

        # fill edges from bottom up until no solid connection
        image = self.manipulation.fill_image(image.copy())
        image = self.manipulation.smooth(image.copy())
        image, x_coordinate, y_coordinate = self.manipulation.getHighestCoordinate(
            image, int(self.width / 2), self.height)

        # create image with furthest possible path with original added
        return cv.addWeighted(img, .7, image, 0.4, 0)

    def determine_location(self, rock_edge, mine_edge):
        if self.robot.goal.line_detection(rock_edge) and self.robot.goal.line_detection(mine_edge):
            if self.robot.start is not True:
                waste = None
                # TODO: talk
            self.robot.start = True
            self.robot.rock_field = False
            self.robot.mining_area = False

        elif not self.robot.goal.line_detection(rock_edge) and self.robot.goal.line_detection(mine_edge):
            if self.robot.rock_field is not True:
                waste = None
                # TODO: talk
            self.robot.start = False
            self.robot.rock_field = True
            self.robot.mining_area = False

        elif not self.robot.goal.line_detection(mine_edge) and self.robot.goal.line_detection(rock_edge):
            if self.robot.rock_field is not True:
                waste = None
                # TODO: talk
            self.robot.start = False
            self.robot.rock_field = True
            self.robot.mining_area = False

        else:
            if self.robot.mining_area is not True:
                waste = None
                # TODO: talk
            self.robot.start = False
            self.robot.rock_field = False
            self.robot.mining_area = True

    def detect_ice(self, frame):
        """
        This function uses blob detection and only considers location of hand.
        :param frame: current frame in consideration.
        :return:
        """

        self.robot.arm_in_cam_view()  # get arm into position
        if self.robot.goal.detect_ice(frame) is True:
            time.sleep(5)  # wait 5 seconds
            self.robot.deliver = True  # Prompts robot to deliver ice
            self.robot.mine = False

    def detect_bin(self, frame):
        """
        This function detects the correct bins by blob detection.
        :param frame: current frame.
        :return:
        """
        if self.robot.goal.detect_bin(frame):
            # TODO: Move towards points
            # TODO: Drop in bin if size of bin is .... (will probably need to be done in bin_ice_detection)
            # TODO: If size < ... then move towards box ... else drop...
            self.robot.drop()

    def orientate(self):
        """
        TODO: Create a function for robot to orientate itself (if it is not facing the correct direction).
        Maybe by finding the color of its goal (it is on the boxes). Once found square up with box, and turn around???
        :return:
        """
        if self.robot.mining_area:  # finds where start is if in mining area by bins.
            if self.robot.goal.detect_bin():
                return True
            else:
                return False


class Robot:
    """
    Robot class is used to control robot movements and commands. It uses Frame class heavily.
    """

    def __init__(self, goal):
        # variable to determine is robot has completed its task
        self.finsihed = False

        # variables to track robots location
        self.start = True
        self.mining_area = False
        self.rock_field = False

        # variables to track robots actions
        self.mine = True
        self.deliver = False

        self.face = FaceDetection()
        self.move = robot_control.MoveRobot()

        self.goal = Goal(goal)  # variable to track robots goal. String that is either S, M, or L

        # self.frame = Frame()  # variable to contain instance of Frame class

    # @staticmethod
    # def robot_talk(what_to_speak):
    #     """
    #     A function that allows the robot to speak a given string.
    #     :param what_to_speak: String, this gives the robot the correct response.
    #     :return:
    #     """
    #     IP = '10.200.47.148'  # value needs to be updated to phone in use
    #     PORT = 5010
    #     speak = client.ClientSocket(IP, PORT)
    #     speak.sendData(what_to_speak)


driver = Frame()
driver.run()

