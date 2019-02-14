import numpy as np
import cv2 as cv
from numpy.distutils.fcompiler import none

image = none
trackbar_values = {'Red Min': 0, 'Red Max': 0, 'Blue Min': 0, 'Blue Max': 0, 'Green Min': 0,
                   'Green Max': 0}  # RBG Min/Max


def show_unfiltered(image):
    cv.namedWindow("Unfiltered video")  # creates unfiltered video from webcam
    cv.moveWindow("Unfiltered video", 0, 20)

    cv.imshow("Unfiltered video", image)  # sHows the unfiltered video


def show_hsv_values(event, x, y, flags, params):  # Method for mouse clicks on HSV image

    if event is cv.EVENT_LBUTTONDOWN:  # Enters if Left button is clicked
        print("HSV value of location x:", x, "y:", y, "Hue:", params[y, x][0],  # Prints location and HSV values
              "Saturation:", params[y, x][1], "Value:", params[y, x][2])


def show_hsv(image):
    param = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image from BGR to HSV
    cv.namedWindow("HSV")  # Creatues window for HSV conversion
    cv.moveWindow("HSV", 643, 20)
    create_trackbar(param)
    cv.imshow("HSV", param)  # Shows the image
    cv.setMouseCallback("HSV", show_hsv_values, param)  # Passes in HSV image during mouse click



def show_cam():
    # Screen capture code provided by Hunter Lloyd https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/View
    capture = cv.VideoCapture(0)
    while True:
        status, image = capture.read()  # Reads in the capture
        show_unfiltered(image)
        create_trackbar(image)
        show_hsv(image)

        k = cv.waitKey(1)
        if k == 27:
            break



# TODO Using Sliders create scalers for the min and max values you want to tracka Scalar will be a numpy array (np.array) that takes 3 values for minH, minS, and minV.......then a second scalar to catch the other three Max values create 3 trackbars, createTrackbar with callback methods to set your six variables
# def set_slider_value(position):
#     values[position]


def set_trackbar_values(val):
    pass


def create_trackbar(hsv):
    cv.namedWindow("HSV")
    cv.moveWindow("HSV", 0, 663)

    cv.createTrackbar('Hue Min', "HSV", 0, 180,
                      set_trackbar_values)
    cv.createTrackbar('Hue Max', "HSV", 0, 180, set_trackbar_values)
    cv.createTrackbar('Saturation Min', "HSV", 0, 255,
                      set_trackbar_values)  # TODO add call back value
    cv.createTrackbar('Saturation Max', "HSV", 0, 255,
                      set_trackbar_values)  # TODO add call back value
    cv.createTrackbar('Value Min', "HSV", 0, 255,
                      set_trackbar_values)  # TODO add call back value
    cv.createTrackbar('Value Max', "HSV", 0, 255,
                      set_trackbar_values)

    hue_min = cv.getTrackbarPos('Hue Min', "HSV")
    hue_max = cv.getTrackbarPos('Hue Max', "HSV")
    saturation_min = cv.getTrackbarPos('Saturation Min', "HSV")
    saturation_max = cv.getTrackbarPos('Saturation Max', "HSV")
    value_min = cv.getTrackbarPos('Value Min', "HSV")
    value_max = cv.getTrackbarPos('Value Max', "HSV")
    kernel = np.ones((5, 5), np.uint8)  # dialates
    minS = np.array([hue_min, saturation_min, value_min])
    maxS = np.array([hue_max, saturation_max, value_max])
    mask = cv.inRange(hsv, minS, maxS)
    mask = cv.dilate(mask, kernel, iterations=1)
   # cv.imshow('HSV', hsv)  # Shows the image
    cv.imshow("Mask", mask)


# TODO Us the OpenCV inRange method to find the values between the scalars from HSV image and the result will go to a grayscale image (make it a binary image, white/black).

# TODO Dilate, erode the grayscale image to get a better representation of the object you are tracking.

# TODO Display the original image and the binary image where everything is black except for the object you are tracking. The tracked object will be white.


show_cam()
cv.destroyAllWindows()
