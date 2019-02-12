import numpy as np
import cv2 as cv
from numpy.distutils.fcompiler import none


def show_unfiltered(image):
    cv.namedWindow("Unfiltered video")  # creates unfiltered video from webcam
    cv.moveWindow("Unfiltered video", 0, 20)
    cv.imshow("Unfiltered video", image)  # sHows the unfiltered video


def show_hsv(image):
    param = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image from BGR to HSV
    cv.namedWindow("HSV")  # Creatues window for HSV conversion
    cv.moveWindow("HSV", 643, 20)
    cv.imshow("HSV", param)  # Shows the image


def show_cam():
    # Screen capture code provided by Hunter Lloyd https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/View
    capture = cv.VideoCapture(0)
    status, image = capture.read()  # Reads in the capture
    running_average = np.float32(image)
    while True:
        status, image = capture.read()  # Reads in the capture
        cv.namedWindow('Grey Blank Image')
        grey_scale = cv.cvtColor(image, cv.COLOR_BGR2GRAY)  # Converts image to grey scale
        cv.imshow('Grey Blank Image', grey_scale)  # Shows grey scale

        cv.namedWindow('Blurred Image')
        # Blurred code source from https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
        blurred = cv.GaussianBlur(image, (7, 7), 0) # Blurs the image with Gaussian Blur function
        cv.imshow('Blurred Image', blurred)
        cv.accumulateWeighted(image, running_average, .320)


        show_unfiltered(image)
        show_hsv(image)

        k = cv.waitKey(1)
        if k == 27:
            break


# TODO Using Sliders create scalers for the min and max values you want to tracka Scalar will be a numpy array (np.array) that takes 3 values for minH, minS, and minV.......then a second scalar to catch the other three Max values create 3 trackbars, createTrackbar with callback methods to set your six variables
# def set_slider_value(position):
#     values[position]


show_cam()
cv.destroyAllWindows()
