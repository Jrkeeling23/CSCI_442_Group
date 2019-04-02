import robot_control
import client
import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


class FaceDetection:
    robot = robot_control.MoveRobot()
    image_width = 0
    image_height = 0
    time_since_talk = 16.0
    time_start = False
    horizontal = 1500
    vertical = 1500
    head_increment_horizontal = 1497
    head_increment_vert = 1497
    # def headRight(self):  # Method to move head right
    #     print("head Right")
    #
    # def headLeft(self):  # Method to move head left
    #     print("head left")
    #
    # def headUp(self):  # Method to move head up
    #     print("head up")
    #
    # def headDown(self):  # Method to move head down
    #     print("head Down")

    def __init__(self):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/V$
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        # robot.center_robot()

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array

            self.image_height, self.image_width, _ = image.shape

            self.detect_face(image.copy())
            # cv.imshow('Face Detection', gray)

            rawCapture.truncate(0)

            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()

    #     eye_cascade = cv.CascadeClassifier('/usr/local/lib/python3.6/dist-packages/cv2/data/haarcascade_eye.xml')
    # ascade = cv.CascadeClassifier('/usr/local/lib/python3.6/dist-pfackages/cv2/data/haarcascade_smile.xml')
    #
    def talk(self):
        print("robot speak")
        IP = '10.200.48.77'
        PORT = 5010
        speak = client.ClientSocket(IP, PORT)
        #speak.start()
        time.sleep(1)
        speak.sendData("Hello Human")   

    def detect_face(self, img):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3947225/View
        # faces = self.face_vars(img.copy())
        time_for_human = 10.0  # Variable setting the time between detecting a new human
        gray = cv.cvtColor(img.copy(), cv.COLOR_BGR2GRAY)

        face_cascade = cv.CascadeClassifier(
            'haarcascade_frontalface_default.xml')

        faces = face_cascade.detectMultiScale(gray, 1.09, 10)
        # faces = []
        # faces = face_cascade.detectMultiScale(gray, 1.9, 5)
        # print(faces)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                self.center(x, y, w, h)
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                roi_gray = gray[y:y + h, x:x + w]
                roi_color = img[y:y + h, x:x + w]
                # Enters if human hasn't been found before or the time is greater than selected amount to find new human.
                # cv.imshow('Face Detection', img)
                if (time.time() - self.time_since_talk) > time_for_human and w > 100 or not self.time_start:
                    self.time_start = True
                    # print("Hello Human")
                    self.talk()

                    self.time_since_talk = time.time()
                    self.robot.move_wheels("turn", self.horizontal) # Centers the robot on the human
                    self.robot.stop()
                    print("set wheels to turn robot towards person",
                          self.horizontal)
                # enters if we are within the time fram of found human
                # if (time.time() - self.time_since_talk) < time_for_human:
                #     # self.robot.stop()   # stops the wheels of the robot
                #     self.center(x, y, w, h)
        elif (time.time() - self.time_since_talk) > time_for_human:  # Enters to search for human face
            # cv.imshow('Face Detection', img)
            # print("face not found.....Set Head location")
            # print("head value: horiz: ", self.horizontal, " vertical: ", self.vertical)
            self.horizontal += self.head_increment_horizontal
            # head_back = False
            if self.horizontal > 7500:
                self.horizontal = 1510
                self.vertical += self.head_increment_vert
            if self.vertical > 7500:
                self.vertical = 1510
            # if self.horizontal >= 7500:
            #     self.horizontal = 7500
            #     head_back = True
            #     self.vertical += self.head_increment_vert
            # elif self.horizontal <= 1510:
            #     self.horizontal = 1510
            #     self.vertical += self.head_increment_vert
            #     head_back = False
            # if head_back:
            #     self.horizontal -= self.head_increment_horizontal
            # elif not head_back:
            #     self.horizontal += self.head_increment_horizontal
            # if self.vertical > 7500:
            #     self.vertical = 1510
            self.move_head()
        cv.imshow('Face Detection', img)
    

    def center(self, x, y, face_w, face_y):
        left = self.image_width * .55
        right = self.image_width * .45
        up = self.image_height * .45
        down = self.image_height * .55
        x_center = x + (face_w / 2)
        y_center = y + (face_y / 2)
        head_inc = 300
        if x_center > left:
            self.horizontal -= head_inc
        elif x_center < right:
            self.horizontal += head_inc
        if y_center < up:
            self.vertical += head_inc
        elif y_center > down:
            self.vertical -= head_inc
        self.move_head()

        if face_w < 75:
            # self.robot.move_wheels("move", 7000)
            print("Move robot forward.", face_w)
            # print("stop forward movement")
        else:
            self.robot.stop()


    def move_head(self):
        self.robot.move_head(self.horizontal, self.vertical)
        time.sleep(.5)
        # moves = {"right": self.headRight, "left": self.headLeft, "up": self.headUp,
        #          "down": self.headDown}  # ["right", "left", "up", "down"]
        # moves[movement].__call__()

    # def face_vars(self, img):
    #     face_cascade = cv.CascadeClassifier(
    #         '/opencv/data/haarcascades/haarcascade_frontalface_default.xml')
    #     faces = face_cascade.detectMultiScale(img, 1.09, 9)
    #     # if len(faces) < 1:
    #     #     faces = self.search_for_face(face_cascade, gray)

    #     return faces


face = FaceDetection()
