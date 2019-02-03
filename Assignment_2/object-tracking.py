import numpy as np
import cv2
# TODO Capture live video from Webcam
# TODO Display live video
# TODO convert BGR to HSV
# TODO Display the HSV video
# TODO Click on HSV video and capture the HSV values at the location clicked, and a few other local values of the item you are going to track.use a MouseCallback to print the HSV values.
# TODO Using Sliders create scalers for the min and max values you want to tracka Scalar will be a numpy array (np.array) that takes 3 values for minH, minS, and minV.......then a second scalar to catch the other three Max values create 3 trackbars, createTrackbar with callback methods to set your six variables
# TODO Us the OpenCV inRange method to find the values between the scalars from HSV image and the result will go to a grayscale image (make it a binary image, white/black).
# TODO Dilate, erode the grayscale image to get a better representation of the object you are tracking.
# TODO Display the original image and the binary image where everything is black except for the object you are tracking. The tracked object will be white.

cap = cv2.VideoCapture(0)
cv2.namedWindow("Video")
while True:
    status, img = cap.read()
    cv2.imshow("Video", img)
    k = cv2.waitKey(1)
    if k == 27:
        break
cv2.destroyAllWindows()
