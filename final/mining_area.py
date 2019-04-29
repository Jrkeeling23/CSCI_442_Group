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


class TestMine:
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
        self.robot = Robot(goal="l")

    def run(self):
        """
        A function to get frames and run the program.
        :return:
        """
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
    
            self.rawCapture.truncate(0)
            img = frame.array

            if self.robot.mining_area and self.robot.mine:  # grab ice
                if self.status is False:
                    status, image = self.robot.face.detect_face(img)
                    if status is True:
                        self.status = True
                        # TODO: "Hello Human"
                else:
                    # TODO: Ask for color of ice.
                    self.detect_ice(img)

                cv.imshow("image", image)

            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()

    def detect_ice(self, frame):
        """
        This function uses blob detection and only considers location of hand.
        :param frame: current frame in consideration.
        :return:
        """
        self.robot.move.arm_in_cam_view()  # get arm into position
        if self.robot.goal.detect_ice(frame) is True:
            time.sleep(5)  # wait 5 seconds
        else:
            waste = None
            # TODO: Rejects ice with talk


class Robot:
    """
    Robot class is used to control robot movements and commands. It uses Frame class heavily.
    """

    def __init__(self, goal):
        # variable to determine is robot has completed its task
        self.finsihed = False

        # variables to track robots location
        self.start = False
        self.mining_area = True
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


test = TestMine()
test.run()