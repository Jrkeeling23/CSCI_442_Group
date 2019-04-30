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

            if self.robot.start and self.robot.mine:  # move through rock field
                flood_fill_image, x = self.create_furthest_path(img)
                orange_found = self.robot.goal.find_orange_lines(img)
                if orange_found:
                    if x > self.width * .7:
                        self.robot.move.turn_right()
                    elif x < self.width * .3:
                        self.robot.move.turn_left()
                    else:
                        self.robot.move.wheels_forward()
                else:
                    self.move.robot.wheels_forward()
                    self.robot.mining_area = True
                    self.robot.start = False

            if self.robot.start and self.robot.deliver:
                if self.robot.goal.bin_area(img) is False and self.robot.found_bin:  # Bin is out o$
                    self.robot.move.wheels_forward()  # get a little closer, if need be....
                    self.robot.move.arm_in_cam_view()
                    self.robot.move.drop()  # drop into box
                    self.robot.finished = True  # terminate program
                else:
                    self.detect_bin(img)

            if self.robot.mining_area and self.robot.deliver:
                flood_fill_image, x = self.create_furthest_path(img)
                orange_found = self.robot.goal.find_orange_lines(img)
                if orange_found:
                    if x > self.width * .7:
                        self.robot.move.turn_right()
                    elif x < self.width * .3:
                        self.robot.move.turn_left()
                    else:
                        self.robot.move.wheels_forward()
                else:
                    self.move.robot.wheels_forward()
                    self.robot.mining_area = False
                    self.robot.start = True

            if self.robot.mining_area and self.robot.mine:  # grab ice
                if self.status is False:
                    status, image = self.robot.face.detect_face(img)
                    if status is True:
                        self.status = True
                        # TODO: "Hello Human"
                else:
                    # TODO: Ask for color of ice.
                    self.detect_ice(img)

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
        orange_lower = np.array([0, 50, 225])
        orange_upper = np.array([30, 255, 255])

        img = cv.blur(img, (5, 5))  # blur initial frame for edge detection
        hsv = cv.cvtColor(img.copy(), cv.COLOR_BGR2HSV)  # convert to HSV to do object detection

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
        return cv.addWeighted(img, .7, image, 0.4, 0), x_coordinate

    def detect_ice(self, frame):
        """
        This function uses blob detection and only considers location of hand.
        :param frame: current frame in consideration.
        :return:
        """
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

    def detect_bin(self, frame):
        """
               This function detects the correct bins by blob detection.
               :param frame: current frame.
               :return:
               """
        if self.robot.goal.bin_area(frame) is False:  # if cannot detect bin, check right
            self.robot.move.turn_right_90()

            if self.robot.goal.bin_area(frame) is False:  # Turn back 180 degrees
                self.robot.move.turn_left_90()
                self.robot.move.turn_left_90()
        else:
            self.robot.found_bin = True
            if self.robot.goal.current_x >= (self.width * .7):
                self.robot.move.turn_right()
            elif self.robot.goal.current_x <= (self.width * .3):
                self.robot.move.turn_left()
            else:
                self.robot.move.wheels_forward()
                print("forward")


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
        self.mine = True
        self.deliver = False

        self.face = FaceDetection()
        self.move = robot_control.MoveRobot()

        self.goal = Goal(goal)  # variable to track robots goal. String that is either Pink, Green, or Orange.


driver = Frame()
driver.run()
