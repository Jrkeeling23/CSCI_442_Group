import numpy as np
import cv2 as cv
from numpy.distutils.fcompiler import none

image = none


def show_unfiltered(image):
    cv.namedWindow("Unfiltered video")  # creates unfiltered video from webcam
    cv.imshow("Unfiltered video", image)  # sHows the unfiltered video


def show_hsv_values(event, x, y, flags, params): # Method for mouse clicks on HSV image
    if event is cv.EVENT_LBUTTONDOWN:   # Enters if Left button is clicked
        print("HSV value of location x:", x, "y:", y, "Hue:", params[y, x][0],  # Prints location and HSV values
              "Saturation:", params[y, x][1], "Value:", params[y, x][2])


def show_hsv(image):
    param = cv.cvtColor(image, cv.COLOR_BGR2HSV) # Converts the image from BGR to HSV
    cv.namedWindow("HSV")  # Creatues window for HSV conversion
    cv.imshow("HSV", param) # Shows the image
    cv.setMouseCallback("HSV", show_hsv_values, param)  # Passes in HSV image during mouse click

    # print(click)
    # if click:
    #     print(cv.HSV)


def show_cam():
    # Screen capture code provided by Hunter Lloyd https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/View
    capture = cv.VideoCapture(0)

    while True:
        status, image = capture.read()  # Reads in the capture
        show_unfiltered(image)
        show_hsv(image)
        # cv.imshow("HSV", cv.cvtColor(image, cv.COLOR_BGR2HSV))    # shows video filtered by hsv
        k = cv.waitKey(1)
        if k == 27:
            break


show_cam()
cv.destroyAllWindows()

# TODO Display the HSV video
# TODO Click on HSV video and capture the HSV values at the location clicked, and a few other local values of the item you are going to track.use a MouseCallback to print the HSV values.
# TODO Using Sliders create scalers for the min and max values you want to tracka Scalar will be a numpy array (np.array) that takes 3 values for minH, minS, and minV.......then a second scalar to catch the other three Max values create 3 trackbars, createTrackbar with callback methods to set your six variables
# TODO Us the OpenCV inRange method to find the values between the scalars from HSV image and the result will go to a grayscale image (make it a binary image, white/black).
# TODO Dilate, erode the grayscale image to get a better representation of the object you are tracking.
# TODO Display the original image and the binary image where everything is black except for the object you are tracking. The tracked object will be white.
