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


class Frame:

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


driver = Frame()
driver.run()
