import threading

import numpy as np
import cv2 as cv
import time
class FaceDetection:
    image_width = 0
    image_height = 0
    time_since_talk = 16.0
    time_start = False
    horizontal = 1500
    vertical = 1500
    head_increment_horizontal = 1497
    head_increment_vert = 1497
    face_cascade = cv.CascadeClassifier(
        'haarcascade_frontalface_default.xml')

    def __init__(self):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/View
        cap = cv.VideoCapture(0)
        # cv.namedWindow("Video")
        while True:
            status, image = cap.read()
            self.image_height, self.image_width, _ = image.shape

            self.detect_face(image)
            # cv.imshow("Video", img)
            cv.imshow('Face Detection', image)
            k = cv.waitKey(1)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        cv.destroyAllWindows()

    #     eye_cascade = cv.CascadeClassifier('/usr/local/lib/python3.6/dist-packages/cv2/data/haarcascade_eye.xml')
    # ascade = cv.CascadeClassifier('/usr/local/lib/python3.6/dist-packages/cv2/data/haarcascade_smile.xml')
    #

        #
        # def talk(self):
        #     print("robot speak")
        #     IP = '10.200.48.77'
        #     PORT = 5010
        #     speak = client.ClientSocket(IP, PORT)
        #     # speak.start()
        #     time.sleep(1)
        #     speak.sendData("Hello Human")

    def detect_face(self, img):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3947225/View
        # faces = self.face_vars(img.copy())
        time_for_human = 5.0  # Variable setting the time between detecting a new human
        gray = cv.cvtColor(img.copy(), cv.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.09, 10)
        # faces = []
        # faces = face_cascade.detectMultiScale(gray, 1.9, 5)
        # print(faces)
        if len(faces) > 0:
            if (time.time() - self.time_since_talk) > time_for_human or not self.time_start:
                self.time_start = True
                print("Hello Human")
                # self.time_since_talk = time.time()

            #                    self.talk()
            for (x, y, w, h) in faces:
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                self.time_since_talk = time.time()
                self.center(x, y, w, h)

        else:  # Enters to search for human face

            self.horizontal += self.head_increment_horizontal
            if self.horizontal > 7500:
                self.horizontal = 1510
                self.vertical += self.head_increment_vert
            if self.vertical > 7500:
                self.vertical = 1510
            self.move_head()

    def center(self, x, y, face_w, face_y):
        left = self.image_width * .55
        right = self.image_width * .45
        up = self.image_height * .45
        down = self.image_height * .55

        x_center = x + (face_w / 2)
        y_center = y + (face_y / 2)
        head_inc = 300
        move_needed = False

        if x_center > left:
            move_needed = True
            self.horizontal -= head_inc
            print("move left")
        elif x_center < right:
            move_needed = True
            print("move right")

            self.horizontal += head_inc
        if y_center < up:
            move_needed = True
            print("move up")

            self.vertical += head_inc
        elif y_center > down:
            move_needed = True
            print("move down")

            self.vertical -= head_inc
        if move_needed:
            threading.Thread(target=self.move_head).start()
        else:
            print("centered")

        if face_w < 150:
            # self.robot.move_wheels("move", 7000)
            threading.Thread(target=self.move_forward).start()



    def move_head(self):
#        self.robot.move_head(self.horizontal, self.vertical)
        pass
        #time.sleep(.5)
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
    def move_forward(self):
        print("Move robot forward.")
        time.sleep(1)


threading.Thread(target=FaceDetection).start()

