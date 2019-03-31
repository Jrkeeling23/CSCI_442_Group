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

    def headRight(self):  # Method to move head right
        print("head Right")

    def headLeft(self):  # Method to move head left
        print("head left")

    def headUp(self):  # Method to move head up
        print("head up")

    def headDown(self):  # Method to move head down
        print("head Down")

    def __init__(self):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/V$
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))
        time.sleep(0.1)

        robot.center_robot()

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            self.image_height, self.image_width, _ = image.shape
            self.detect_face(image.copy())

            k = cv.waitKey(1) & 0xFF
            if k == ord('q'):
                break
        cv.destroyAllWindows()

    #     eye_cascade = cv.CascadeClassifier('/usr/local/lib/python3.6/dist-packages/cv2/data/haarcascade_eye.xml')
    # ascade = cv.CascadeClassifier('/usr/local/lib/python3.6/dist-packages/cv2/data/haarcascade_smile.xml')
    #

    def detect_face(self, img):
        # Sourced from https://ecat.montana.edu/d2l/le/content/524639/viewContent/3947225/View
        face_cascade, gray, faces = self.face_vars(img)

        # print(faces)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                roi_gray = gray[y:y + h, x:x + w]
                roi_color = img[y:y + h, x:x + w]
                if (time.time() - self.time_since_talk) > 15.0 and w > 100 or not self.time_start:
                    self.time_start = True
                    print("Hello Human")
                    self.time_since_talk = time.time()
                    print("set wheels to turn robot towards person", self.horizontal)
                if (time.time() - self.time_since_talk) < 15.0:
                    self.move_robot(x, y, w, h)
        elif (time.time() - self.time_since_talk) > 15.0:
            # print("face not found.....Set Head location")
            # print("head value: horiz: ", self.horizontal, " vertical: ", self.vertical)
            self.horizontal += 100
            if self.horizontal > 7500:
                self.horizontal = 1500
                self.vertical += 100
                if self.vertical > 7500:
                    self.vertical = 1500

        cv.imshow('Face Detection', img)

    def move_robot(self, x, y, face_w, face_y):
        left = self.image_width * .7
        right = self.image_width * .3
        up = self.image_height * .3
        down = self.image_height * .7
        x_center = x + (face_w / 2)
        y_center = y + (face_y / 2)
        if face_w < 75:
            print("Move robot forward.", face_w)
            print("stop forward movement")

        if x_center > left:
            self.horizontal -= 50
            self.move_head("left")
        elif x_center < right:
            self.horizontal += 50
            self.move_head("right")
        if y_center < up:
            self.vertical += 50
            self.move_head("up")
        elif y_center > down:
            self.horizontal -= 50
            self.move_head("down")

    def move_head(self, movement):
        moves = {"right": self.headRight, "left": self.headLeft, "up": self.headUp,
                 "down": self.headDown}  # ["right", "left", "up", "down"]
        moves[movement].__call__()

    def face_vars(self, img):
        face_cascade = cv.CascadeClassifier(
            '/usr/local/lib/python3.6/dist-packages/cv2/data/haarcascade_frontalface_default.xml')
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.09, 9)
        # if len(faces) < 1:
        #     faces = self.search_for_face(face_cascade, gray)

        return face_cascade, gray, faces


face = FaceDetection()
