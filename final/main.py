import cv2 as cv
import numpy as np
import imageManipulation
import makeMoves
import robot_control
import client
import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
import socket
import time
import queue

"""
TODO: 
Find ranges for ice/goal colors.
Give robot class Driver instance.
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

        self.robot = Robot(goal="l")

    def run(self):
        """
        A function to get frames and run the program.
        :return:
        """
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            img = frame.array

            self.robot.orientate()  # robot must find where it is at
            # TODO: turn around do a 360 now. Robot orientates based off of bins.

            if self.robot.mine:  # is robot is needs to mine, needs to get path to mine
                self.robot.get_path()

            elif self.robot.deliver:  # if robot needs to deliver, call function.
                self.robot.deliver_ice()

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

    def face_detection(self):
        """
        TODO: Create a function that detects face. Robot will use this from find_human function.
        :return:
        """

    def center(self):
        """
        TODO: Create function to center robot to path found. Will be called when robot finds a path
        :return:
        """

    def detect_ice(self, goal):
        """
        TODO: Create function to find blob within hand region of robot
        :param goal: Gives robot understanding of what ice to be detecting
        :return:
        """

    def detect_bin(self, goal):
        """
        TODO: Create function to find bin that corresponds to ice color
        :param goal: Gives robot understanding of what color of bin to be detecting
        :return:
        """

    def orientate(self):
        """
        TODO: Create a function for robot to orientate itself (if it is not facing the correct direction).
        Maybe by finding the color of its goal (it is on the boxes). Once found square up with box, and turn around???
        :return:
        """


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

        self.goal = goal  # variable to track robots goal. String that is either S, M, or L

        self.frame = Frame()  # variable to contain instance of Frame class

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


    def grab(self):
        """
        Function that controls robot movement to grab ice.
        area (where arm will be) to detect if ice is in robot hands.
        :return:
        """
        # TODO: Squeeze Hand

    def deliver_ice(self):
        """
        TODO: Create function to deliver ice in correct box. Entails robot arm movement to drop ice.
        :return:
        """
        self.orientate()  # squares up with box

        while True:
            if True:  # TODO: Some statement that determines if it is at bin (probably method).
                self.deliver = False  # robot has delivered
            # TODO: Drop ice in bin
            self.get_path()



    def robot_center(self):
        """
        TODO: Function to center robot. Uses Frame's center function
        :return:
        """

    def move_head(self):  # function that moves the robot head.
        """
        Function to operate head movements.
        :return:
        """
        self.robot.move_head(self.horizontal, self.vertical)

    def move_forward(self):
        """
        Function to move robot forwards.
        :return:
        """
        # self.robot.move_wheels("move", self.wheels_value)
        self.robot.wheels_forward()
        print("move wheels")

    def move_back(self):
        """
        Function to move robot backwards.
        :return:
        """
        # self.robot.move_wheels("move", self.wheels_value)
        self.robot.wheels_forward()
        print("move wheels")

    def turn_right(self):
        """
        Function to turn robot right.
        :return:
        """
        self.robot.turn_right()
        # self.robot.move_wheels("turn", self.turn_value)
        print("turn wheels")

    def turn_left(self):
        """
        Function to turn robot left
        :return:
        """
        self.robot.turn_left()
        # self.robot.move_wheels("turn", self.turn_value)
        print("turn wheels")

    def move_arm(self):
        """
        TODO: Figure out fixed position to move arm when ready to grab.
        :return:
        """

    def increment_Movement(self, move, min, max, inc1, inc2):
        """
        TODO: This is copy and pasted from Assignment 6, will need to adjust for this program...
        :param move: String for robot movement command.
        :param min: ?
        :param max: ?
        :param inc1: ?
        :param inc2: ?
        :return:
        """
        moves = {"head": self.move_head, "forward": self.move_forward, "backward": self.move_back,
                 "left": self.turn_left, "right": self.turn_right}

        if move == "head":
            self.horizontal = self.search_for_face_inc
            if self.horizontal > max:  # Checks if head has reached farthest right value
                self.search_for_face_inc = max
                self.search_for_face_up = False  # Sets to false to head the other direction
                # Sets the face value iin case it is greater than 7500
                self.horizontal = self.search_for_face_inc
                self.vertical += inc2  # Increments the vertical position
            elif self.horizontal < min:  # Checks if head is in the farthest left postion
                self.search_for_face_inc = min
                # Sets true to start heading the other way.
                self.search_for_face_up = True
                self.horizontal = self.search_for_face_inc  # Sets incase head is less than 1519
                self.vertical += inc2  # Increments the vertical postition

            if self.vertical > max:  # Resets to bottom vertical position
                self.vertical = min
            if self.search_for_face_up:
                self.search_for_face_inc += inc1
            else:
                self.search_for_face_inc -= inc1
        elif move == "forward":
            self.wheels_value -= inc1
            if self.wheels_value > max:
                self.wheels_value = max
            elif self.wheels_value < min:
                self.wheels_value = min
        elif move == "backward":
            self.wheels_value += inc1
            if self.wheels_value > max:
                self.wheels_value = max
            elif self.wheels_value < min:
                self.wheels_value = min
        elif move == "right":
            self.turn_value -= inc1
            if self.turn_value > max:  # Checks if head has reached farthest lef value
                self.turn_value = max
            elif self.turn_value < min:  # Checks if head is in the farthest right postion
                self.turn_value = min
        elif move == "left":
            self.turn_value += inc1
            if self.turn_value > max:  # Checks if head has reached farthest left value
                self.turn_value = max
            elif self.turn_value < min:  # Checks if head is in the farthest right postion
                self.turn_value = min

        moves[move].__call__()


driver = Frame()
driver.run()

#     def __init__(self):
#         self.manipulation = imageManipulation.ImageManipulation()
#         self.cap = cv.VideoCapture(0)
#         status, img = self.cap.read()
#         #img = cv.imread('im2.jpg')
#
#         self.height, self.width, _ = img.shape
#         self.move = makeMoves.Move(self.width, self.height)
#
#     def run(self):
#         while True:
#             status, img = self.cap.read()
#             #img = cv.imread('im2.jpg')
#             image = self.manipulation.edge_detection(img.copy())
#             image = self.manipulation.fill_image(image)
#             image = self.manipulation.smooth(image)
#             image, x_coordinate, y_coordinate = self.manipulation.getHighestCoordinate(image, int(self.width / 2), self.height)
#             # self.move.decide_move(x_coordinate, y_coordinate)
#             overlayed = cv.addWeighted(img, .7, image, 0.4, 0)  # Overlays the path on the original image
#             cv.imshow("Path", overlayed)
#             k = cv.waitKey(1)
#             if k == 27:
#                 break
#         cv.destroyAllWindows()
#
#
#
