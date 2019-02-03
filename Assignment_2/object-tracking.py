import numpy as np
import cv2 as cv


def show_unfiltered(img):
    cv.namedWindow("Unfiltered video")  # creates unfiltered video from webcam
    cv.imshow("Unfiltered video", img)  # sHows the unfiltered video


def show_hsv_values():
    if cv.EVENT_LBUTTONDBLCLK:
        print("test")


def show_hsv(img):
    cv.namedWindow("HSV")  # Creatues window for HSV conversion
    cv.imshow("HSV", cv.cvtColor(img, cv.COLOR_BGR2HSV))
    cv.setMouseCallback("HSV", show_hsv_values)

    # print(click)
    # if click:
    #     print(cv.HSV)


def show_cam():
    # Screen capture code provided by Hunter Lloyd https://ecat.montana.edu/d2l/le/content/524639/viewContent/3826523/View
    capture = cv.VideoCapture(0)

    while True:
        status, img = capture.read()  # Reads in the capture
        show_unfiltered(img)
        show_hsv(img)
        # cv.imshow("HSV", cv.cvtColor(img, cv.COLOR_BGR2HSV))    # shows video filtered by hsv
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
