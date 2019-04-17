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


class Driver:
    def __init__(self):
        """
        TODO: Maybe add states to keep track of what the robot needs to do (go to mining area) and to keep track of
        where the robot is located (for speaking purposes).
        Maybe have booleans such as mine (go to mining area), deliver (has ice and needs to deliver) for actions
        Have booleans start and mining_area to keep track where the robot is (for talking purposes)
        And a variable called goal which correlates with big, medium, and small to know what color our ice is.
        """
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

    def run(self):
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            img = frame.array

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

    def robot_talk(self, what_to_speak):
        """
        TODO: similar to assignment 6, create function for robot to speak when it finds a human, crosses over boundaries
        (mining area etc.).
        :param what_to_speak: This gives the robot the correct response.
        :return:
        """

    def find_human(self):
        """
        TODO: create function to find human (once in mining area).
        :return:
        """

    def grab_ice(self):
        """
        TODO: Create function to grab ice. This entails robot arm movement (maybe its own function), and blob detection
        area (where arm will be) to detect if ice is in robot hands.
        :return:
        """

    def deiliver_ice(self):
        """
        TODO: Create function to deliver ice in correct box. Entails robot arm movement to drop ice.
        :return:
        """

    def orientate(self):
        """
        TODO: Create a function for robot to orientate itself (if it is not facing the correct direction).
        Maybe by finding the color of its goal (it is on the boxes). Once found square up with box???
        :return:
        """


driver = Driver()
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
