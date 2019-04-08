import robot_control
#import client
import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading


class FaceDetection:
    image_width = 0
    image_height = 0
    time_since_talk = 16.0
    time_start = False
    horizontal = 1500
    vertical = 1500
    head_increment_horizontal = 1996
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
        camera.framerate = 15
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
        # IP = '10.200.48.77'
        # PORT = 5010
        # speak = client.ClientSocket(IP, PORT)
        # # speak.start()
        # time.sleep(1)
        # speak.sendData("Hello Human")
        print("hello human")
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
                self.robot_centered = False
            for (x, y, w, h) in faces:  # Loops over faces (should be only one)
                self.time_since_talk = time.time()  # resets the clock since a human has been found.
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                self.center(x, y, w, h)  # Calls the function to center the face.
                if not robot_control:
                    if self.horizontal < 5800: # Makes robot face the human.
                        self.increment_Movement("left", 2110, 7400, self.turn_inc, 0)
                    elif self.horizontal > 6200:
                        self.increment_Movement("right", 2110, 7400, self.turn_inc, 0)
                    else:
                        self.robot.stop()
                        self.robot_centered = True
        else:
            # # Adjust head increments to find a face.
            self.horizontal += self.head_increment_horizontal
            if self.horizontal > 7500:
                self.horizontal = 1510
                self.vertical += self.head_increment_vert
            if self.vertical > 7500:
                self.vertical = 1510
            self.move_head()
            time.sleep(1)
           # self.search_for_face()
        #    self.increment_Movement("head", 1510, 7500, 1996, 1996)
        #    time.sleep(.4)

    def center(self, x, y, face_w, face_y):  # Function to center head and move robot towards human.
        # first four variables define what is considered outside of center of image.
        left = self.image_width * .6
        right = self.image_width * .4
        up = self.image_height * .4
        down = self.image_height * .6
        # Get the x and y values for the center of the box surrounding the face.
        x_center = x + (face_w / 2)
        y_center = y + (face_y / 2)
        head_inc = 175  # variable adjust how much the head moves each iteration
        move_needed = False  # Boolean for later function to decide if moving the head is needed.

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
            threading.Thread(target=self.move_head).start()  # moves the head

        if face_w < 75:  # 75 is the value to decide if the robot needs to move forward or not.
            print("move forward")
            #threading.Thread(target=self.robot.move_wheels("move", 7000)).start()
        elif face_w > 250:
            self.increment_Movement("backward", 1510, 500, self.wheels_inc, 0)
        elif self.wheels_value != 6000:
            self.wheels_value = 6000
            self.robot.stop()

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

    def move_wheels(self):
        #self.robot.move_wheels("move", self.wheels_value)
        print("move wheels")
    def turn_wheels(self):
        #self.robot.move_wheels("turn", self.turn_value)
        print("turn wheels")
    def increment_Movement(self, move, min, max, inc1, inc2):  # iteratively moves robot.
        moves = {"head": self.move_head, "forward": self.move_wheels, "backward": self.move_wheels,
                 "left": self.turn_wheels, "right": self.turn_wheels}

        if move == "head":
            self.horizontal = self.search_for_face_inc
            if self.horizontal > max:  # Checks if head has reached farthest right value
                self.search_for_face_inc = max
                self.search_for_face_up = False  # Sets to false to head the other direction
                self.horizontal = self.search_for_face_inc  # Sets the face value iin case it is greater than 7500
                self.vertical += inc2  # Increments the vertical position
            elif self.horizontal < min:  # Checks if head is in the farthest left postion
                self.search_for_face_inc = min
                self.search_for_face_up = True  # Sets true to start heading the other way.
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

        threading.Thread(target=moves[move].__call__())


threading.Thread(target=FaceDetection).start()  # Starts the program on it's own thread.
