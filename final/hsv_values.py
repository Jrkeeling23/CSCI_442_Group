
import numpy as np

import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

image = None
trackbar_values = {'Red Min': 0, 'Red Max': 0, 'Blue Min': 0, 'Blue Max': 0, 'Green Min': 0,
                   'Green Max': 0}  # RBG Min/Max


def show_unfiltered(image):
    cv.namedWindow("Unfiltered video")  # creates unfiltered video from webcam
    cv.imshow("Unfiltered video", image)  # sHows the unfiltered video


def show_hsv_values(event, x, y, flags, params):  # Method for mouse clicks on HSV image

    if event is cv.EVENT_LBUTTONDOWN:  # Enters if Left button is clicked
        print("HSV value of location x:", x, "y:", y, "Hue:", params[y, x][0],  # Prints location and HSV values
              "Saturation:", params[y, x][1], "Value:", params[y, x][2])


def show_hsv(image):
    param = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image from BGR to HSV
    cv.namedWindow("HSV")  # Creatues window for HSV conversion
    # cv.moveWindow("HSV", 643, 20)
    create_trackbar(param)
    cv.imshow("HSV", param)  # Shows the image
    cv.setMouseCallback("HSV", show_hsv_values, param)  # Passes in HSV image during mouse click


def show_cam():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        show_hsv(img)


        rawCapture.truncate(0)
        k = cv.waitKey(1) & 0xFF
        if k == ord('q'):
            break
    cv.destroyAllWindows()



def set_trackbar_values(val):
    pass


### Creates trackbar for hsv values to detect object
def create_trackbar(hsv):
    cv.namedWindow("HSV")

    # cv.135Window("HSV", 0, 663)  # move window to a desirable spot

    # cv.moveWindow("HSV", 0, 663)

    # trackbars for max and mins for each hsv channel
    # cv.createTrackbar('Hue Min', "HSV", 0, 180,
    #                   set_trackbar_values)
    # cv.createTrackbar('Hue Max', "HSV", 0, 180, set_trackbar_values)
    # cv.createTrackbar('Saturation Min', "HSV", 0, 255,
    #                   set_trackbar_values)
    # cv.createTrackbar('Saturation Max', "HSV", 0, 255,
    #                   set_trackbar_values)
    # cv.createTrackbar('Value Min', "HSV", 0, 255,
    #                   set_trackbar_values)
    # cv.createTrackbar('Value Max', "HSV", 0, 255,
    #                   set_trackbar_values)

    cv.createTrackbar('Hue Min', "HSV", 0, 255,
                      set_trackbar_values)
    cv.createTrackbar('Hue Max', "HSV", 0, 255, set_trackbar_values)
    cv.createTrackbar('Saturation Min', "HSV", 0, 255,
                      set_trackbar_values)
    cv.createTrackbar('Saturation Max', "HSV", 0, 255,
                      set_trackbar_values)
    cv.createTrackbar('Value Min', "HSV", 0, 255,
                      set_trackbar_values)
    cv.createTrackbar('Value Max', "HSV", 0, 255,
                      set_trackbar_values)

    # obtain the min and max positions of the sliders
    hue_min = cv.getTrackbarPos('Hue Min', "HSV")
    hue_max = cv.getTrackbarPos('Hue Max', "HSV")
    saturation_min = cv.getTrackbarPos('Saturation Min', "HSV")
    saturation_max = cv.getTrackbarPos('Saturation Max', "HSV")
    value_min = cv.getTrackbarPos('Value Min', "HSV")
    value_max = cv.getTrackbarPos('Value Max', "HSV")
    kernel = np.ones((5, 5), np.uint8)  # dialates
    # create arrays to hold as a scalar for min and max
    minS = np.array([hue_min, saturation_min, value_min])
    maxS = np.array([hue_max, saturation_max, value_max])
    mask = cv.inRange(hsv, minS, maxS)
    mask = cv.dilate(mask, kernel, iterations=1)
    # cv.imshow('HSV', hsv)  # Shows the image
    cv.imshow("Mask", mask)


show_cam()
cv.destroyAllWindows()
