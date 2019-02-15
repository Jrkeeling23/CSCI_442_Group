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
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image from BGR to HSV
        cv.namedWindow("HSV")  # Creatues window for HSV conversion
        cv.moveWindow("HSV", 643, 20)
        cv.imshow("HSV", hsv)  # Shows the image

        cv.namedWindow('Grey Blank Image')
        grey_scale = cv.cvtColor(image, cv.COLOR_BGR2GRAY)  # Converts image to grey scale
        cv.imshow('Grey Blank Image', grey_scale)  # Shows grey scale

        cv.namedWindow('Blurred Image')
        # Blurred code source from https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
        blurred = cv.GaussianBlur(image, (7, 7), 0)  # Blurs the image with Gaussian Blur function
        cv.imshow('Blurred Image', blurred)
        cv.namedWindow('Running Average')

        cv.accumulateWeighted(blurred, running_average,
                              .320)  # Gets the weighted accumulate average between the float32 image and the standard image
        cv.imshow('Running Average', running_average)

        cv.namedWindow('Convert Scale to Abs')
        eight_bit_image = cv.convertScaleAbs(running_average)  # converts the running average to an eight bit image
        cv.imshow('Convert Scale to Abs', eight_bit_image)

        cv.namedWindow('Abs Diff')
        image_difference = cv.absdiff(eight_bit_image,
                                      image)  # Gets the absolute difference between the eight bit image and the original image
        cv.imshow('Abs Diff', image_difference)

        image_difference = cv.cvtColor(image_difference,
                                       cv.COLOR_BGR2GRAY)  # converts the absolute difference image to grey scale
        cv.namedWindow('Grey Scale Abs Diff')
        cv.imshow('Grey Scale Abs Diff', image_difference)

        return_val, image_threshold = cv.threshold(image_difference, 25, 255,
                                                   cv.THRESH_BINARY)  # Thresholds the image above a threshold value above 30. Binary used for following contour conversion
        threshold_blurred = cv.GaussianBlur(image_threshold, (7, 7), 0)  # Blurs the Threshold image
        return_val, image_threshold = cv.threshold(threshold_blurred, 50, 255,
                                                   cv.THRESH_BINARY)  # Thresholds the blurred image again with a threshold value above 200
        cv.namedWindow("Threshold")
        cv.imshow("Threshold", image_threshold)

        # Parts of Find contours code sourced from https://docs.opencv.org/3.1.0/d4/d73/tutorial_py_contours_begin.html
        contour_image = image_threshold
        contours, hierarchy = cv.findContours(image_threshold.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE, ) #RETR_EXTERNAL
        # Draw counters helped from user Mahm00d source: https://stackoverflow.com/questions/34961349/draw-contours-in-opencv-around-recognized-polygon
        cv.drawContours(contour_image, contours, -1, (0, 255, 0), 3)  # Draws the contours
        # if contours:
        #     print(contours)
        for c in contours:
            if cv.contourArea(c)<10000:
                continue
            (x,y,w,h) = cv.boundingRect(c)
            cv.rectangle(image,(x,y), (x+w,y+h), (0,0,255),3)



        cv.namedWindow("Unfiltered video")  # creates unfiltered video from webcam
        cv.moveWindow("Unfiltered video", 0, 20)
        cv.imshow("Unfiltered video", image)  # sHows the unfiltered video

        # show_unfiltered(image)
        # show_hsv(image)

        k = cv.waitKey(1)
        if k == 27:
            break


# TODO Using Sliders create scalers for the min and max values you want to tracka Scalar will be a numpy array (np.array) that takes 3 values for minH, minS, and minV.......then a second scalar to catch the other three Max values create 3 trackbars, createTrackbar with callback methods to set your six variables
# def set_slider_value(position):
#     values[position]

def draw_keypoints(vis, keypoints, color=(0,0,255)):
    for kp in keypoints:
        x,y = kp.pt
        cv.circle(vis,(int(x), int(y),2,color))

show_cam()
cv.destroyAllWindows()
