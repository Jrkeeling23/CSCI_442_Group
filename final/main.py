import cv2 as cv
import imageManipulation
import makeMoves
import robot_control
import client
import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
import socket, time
import queue

class Driver:
    def __init__(self):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/V$
        camera = PiCamera()
        # camera.resolution = (640, 480)
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        self.manipulation = imageManipulation.ImageManipulation()

        self.move = makeMoves.Move(self.width, self.height)


        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            self.image_height, self.image_width, _ = image.shape  # Gets the image size
            self.detect_face(image)

            image = self.manipulation.edge_detection(img.copy())
            image = self.manipulation.fill_image(image)
            image = self.manipulation.smooth(image)
            image, x_coordinate, y_coordinate = self.manipulation.getHighestCoordinate(image, int(self.width / 2), self.height)
            # self.move.decide_move(x_coordinate, y_coordinate)
            overlayed = cv.addWeighted(img, .7, image, 0.4, 0)  # Overlays the path on the original image
            cv.imshow("Path", overlayed)





            cv.imshow('Face Detection', image)
            rawCapture.truncate(0)
            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()



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
# driver = Driver()
# driver.run()
