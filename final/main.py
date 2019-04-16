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

blue_lower = np.array([80,0, 180])
blue_upper = np.array([120, 40, 230])

orange_lower = np.array([10, 0, 150])
orange_upper = np.array([50, 50, 230])


class Driver:
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

            # convert to HSV to do object detection
            hsv = cv.cvtColor(img.copy(), cv.COLOR_BGR2HSV)
            # create image with only blue displayed (as white)
            blue_mask = cv.inRange(hsv, blue_lower, blue_upper)
            # create image with only ornage displayed (as white)
            orange_mask = cv.inRange(hsv, orange_lower, orange_upper)

            # get edges of blue and orange mask to subtract later on **** may not need
            blue_edge = self.manipulation.edge_detection(blue_mask)
            orange_edge = self.manipulation.edge_detection(orange_mask)
            # TODO: add orange and blue edge together, then subtract from overall edge detection picture or add.
            blue_orange = cv.add(blue_mask, orange_mask)
            res = cv.bitwise_and(blue_mask,orange_mask,blue_orange)
            cv.imshow(res, "test")
            # self.image_height, self.image_width, _ = image.shape  # Gets the image size
            # self.detect_face(image)
            # self.height, self.width, _ = img.shape  # Gets the image size

            # self.move = makeMoves.Move(self.width, self.height)
            # print("w ", self.width, "h, ",self.height)
            image = self.manipulation.edge_detection(img.copy())
            image = self.manipulation.fill_image(image)
            image = self.manipulation.smooth(image)
            #image = cv.add(blue_orange, image) # add the white space of orange and blue barriers to flood filled image
            image, x_coordinate, y_coordinate = self.manipulation.getHighestCoordinate(
                image, int(self.width / 2), self.height)
            # self.move.decide_move(x_coordinate, y_coordinate)
            # Overlays the path on the original image

            overlayed = cv.addWeighted(img, .7, image, 0.4, 0)
            #cv.imshow("Path", overlayed)

            # cv.imshow('Face Detection', image)
            self.rawCapture.truncate(0)
            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()


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
