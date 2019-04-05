import robot_control_zeroes
import client
import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
import socket, time
import queue


class FaceDetection:
    image_width = 0
    image_height = 0
    time_since_talk = 16.0
    time_start = False
    horizontal = 1500
    vertical = 1500
    head_increment_horizontal = 1198
    search_for_face_inc = 1510
    search_for_face_up = True
    wheels_value = 6000
    wheels_inc = 50
    turn_inc = 50
    turn_value = 6000
    head_increment_vert = 1996
    robot = robot_control.MoveRobot()
    face_cascade = cv.CascadeClassifier(
        'haarcascade_frontalface_default.xml')

    def __init__(self):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/V$
        camera = PiCamera()
        #camera.resolution = (640, 480)
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            self.image_height, self.image_width, _ = image.shape  # Gets the image size
            self.detect_face(image)
            cv.imshow('Face Detection', image)
            rawCapture.truncate(0)
            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()

    def talk(self):  # Method to call the robot talk function
        # print("robot speak")
        # IP = '10.200.9.122'
        # PORT = 5010
        # speak = client.ClientSocket(IP, PORT)
        # speak.start()
        # time.sleep(1)
        # speak.sendData("Hello Human")
        # print("hello human")
                    

        IP = '10.200.57.202'
        PORT = 5010
        speak = client.ClientSocket(IP, PORT)
        speak.sendData("hello human")

    robot_centered = False

    def detect_face(self, img):  # Method to detect the human face
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3947225/View
        time_for_human = 10.0  # Variable setting the time between detecting a new human
        gray = cv.cvtColor(img.copy(), cv.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.09, 10)
        if len(faces) > 0:  # Enters if a face is found
            if (
                    time.time() - self.time_since_talk) > time_for_human or not self.time_start:  # Enters if a human is found the first time running a program, or a human hasn't been found for the chosen amount of time.
                self.time_start = True  # Tracks the first time a human is found running the program
                self.talk()
                print("vert: ", self.vertical)
                self.robot_centered = False
            else:
                self.time_start = False
            for (x, y, w, h) in faces:  # Loops over faces (should be only one)
                # resets the clock since a human has been found.
                self.time_since_talk = time.time()
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                # Calls the function to center the face.
                move_for = False
                if not self.robot_centered:
                    if self.horizontal <= 5200 and not self.robot_centered:  # Makes robot face the human.
                        # self.increment_Movement(
                       #     "left", 2110, 7400, self.turn_inc, 0)
                        self.robot.turn_right()
                        self.center(x, y, w, h)

                    elif self.horizontal >= 6800 and not self.robot_centered:
                        #    self.increment_Movement(
                     #       "right", 2110, 7400, self.turn_inc, 0)
                        self.robot.turn_left()
                        self.center(x, y, w, h)

                    else:
                        self.robot_centered = True
                        move_for = True
                        print("robot centered")
                    if move_for:

                        if w < 75:  # 75 is the value to decide if the robot needs to move forward or not.
                            print("move forward")
                            self.robot.wheels_forward()
                            #threading.Thread(target=self.robot.move_wheels("move", 7000)).start()
                        elif w > 250:
                           # self.increment_Movement("backward", 1510, 500, self.wheels_inc, 0)
                            self.robot.wheels_backward()
                        self.center(x, y, w, h)


        elif (time.time() - self.time_since_talk) > time_for_human:
            # # Adjust head increments to find a face.
            self.horizontal += self.head_increment_horizontal
            if self.horizontal > 7500:
                self.horizontal = 1510
                self.vertical += self.head_increment_vert
            if self.vertical > 7500:
                self.vertical = 1510
            self.move_head()
            # time.sleep(1)
           # self.search_for_face()
        #    self.increment_Movement("head", 1510, 7500, 1996, 1996)
        #    time.sleep(.4)

    # # Function to center head and move robot towards human.
    def center(self, x, y, face_w, face_y):
        # first four variables define what is considered outside of center of image.
        left = self.image_width * .55
        right = self.image_width * .45
        up = self.image_height * .45
        down = self.image_height * .55
        # Get the x and y values for the center of the box surrounding the face.
        x_center = x + (face_w / 2)
        y_center = y + (face_y / 2)
        head_inc = 400  # variable adjust how much the head moves each iteration
        # Boolean for later function to decide if moving the head is needed.
        move_needed = False

        if x_center > left:  # Checks if robot needs to move head left.

            move_needed = True
            self.horizontal -= head_inc
        elif x_center < right:  # Checks if robot needs to move head right.
            move_needed = True
            self.horizontal += head_inc
        if y_center < up:  # Checks if robot needs to move head up.
            move_needed = True
            self.vertical += head_inc
        elif y_center > down:  # Checks if robot needs to move head down.
            move_needed = True
            self.vertical -= head_inc
        if move_needed:
            min = 1510
            max = 7500
            # Verifies values are within bounds.
            if self.vertical < min:
                self.vertical = min
            elif self.vertical > max:
                self.vertical = max
            if self.horizontal < min:
                self.horizontal = min
            elif self.horizontal > max:
                self.horizontal = max
            self.move_head()  # moves the head

    #     # elif self.wheels_value != 6000:
    #     #     self.wheels_value = 6000
    #     #     self.robot.stop()

    def move_head(self):  # function that moves the robot head.
        self.robot.move_head(self.horizontal, self.vertical)

    # def search_for_face(self): # Searches for face back and forth.
    #     head_inc_value = 599
    #     head_increment_vert = 1198
    #     head_max = 7500
    #     head_min = 1510
    #     self.horizontal = self.search_for_face_inc
    #     if self.horizontal > head_max:  # Checks if head has reached farthest right value
    #         self.search_for_face_inc = head_max
    #         self.search_for_face_up = False  # Sets to false to head the other direction
    #         self.horizontal = self.search_for_face_inc  # Sets the face value iin case it is greater than 7500
    #         self.vertical += head_increment_vert  # Increments the vertical position
    #     elif self.horizontal < head_min:  # Checks if head is in the farthest left postion
    #         self.search_for_face_inc = head_min
    #         self.search_for_face_up = True  # Sets true to start heading the other way.
    #         self.horizontal = self.search_for_face_inc  # Sets incase head is less than 1519
    #         self.vertical += head_increment_vert  # Increments the vertical postition
    #
    #     if self.vertical > head_max:  # Resets to bottom vertical position
    #         self.vertical = head_min
    #     if self.search_for_face_up:
    #         self.search_for_face_inc += head_inc_value
    #     else:
    #         self.search_for_face_inc -= head_inc_value
    #     self.move_head()

    def move_forward(self):
        #self.robot.move_wheels("move", self.wheels_value)
        self.robot.wheels_forward()
        print("move wheels")

    def move_back(self):
            #self.robot.move_wheels("move", self.wheels_value)
        self.robot.wheels_forward()
        print("move wheels")

    def turn_right(self):
        self.robot.turn_right()
        #self.robot.move_wheels("turn", self.turn_value)
        print("turn wheels")

    def turn_left(self):
        self.robot.turn_left()
        #self.robot.move_wheels("turn", self.turn_value)
        print("turn wheels")

    # iteratively moves robot.
    def increment_Movement(self, move, min, max, inc1, inc2):
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


    # def center(self, x, y, face_w, face_y):
    #     # first four variables define what is considered outside of center of image.
    #     left = self.image_width * .55
    #     right = self.image_width * .45
    #     up = self.image_height * .45
    #     down = self.image_height * .55
    #     center_hori = self.image_width * .5
    #     center_vert = self.image_height * .5

    #     # Get the x and y values for the center of the box surrounding the face.
    #     x_center = x + (face_w / 2)
    #     y_center = y + (face_y / 2)
    #     new_x = int(6000 * (x_center / center_hori))
    #     new_y = int(6000 * (y_center / center_vert))

    #     head_inc = 200  # variable adjust how much the head moves each iteration
    #     # Boolean for later function to decide if moving the head is needed.
    #     move_needed = False

    #     if x_center > left:  # Checks if robot needs to move head left.

    #         move_needed = True
    #         self.horizontal = new_x
    #     elif x_center < right:  # Checks if robot needs to move head right.
    #         move_needed = True
    #         self.horizontal =new_x
    #     if y_center < up:  # Checks if robot needs to move head up.
    #         move_needed = True
    #         self.vertical =new_y
    #     elif y_center > down:  # Checks if robot needs to move head down.
    #         move_needed = True
    #         self.vertical = new_y
    #     if move_needed:
    #         min = 1510
    #         max = 7500
    #         # Verifies values are within bounds.
    #         if self.vertical < min:
    #             self.vertical = min
    #         elif self.vertical > max:
    #             self.vertical = max
    #         if self.horizontal < min:
    #             self.horizontal = min
    #         elif self.horizontal > max:
    #             self.horizontal = max
    #         self.move_head()  # moves the head


face = FaceDetection()  # Starts the program on it's own thread.
