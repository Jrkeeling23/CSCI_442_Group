import robot_control
import cv2 as cv
import numpy as np
from picamera import PiCamera
import time
from picamera.array import PiRGBArray


def traverse_pixels(image):  # removes the top 80% of the screen view
    new_image = image.copy()
    height, width, x = new_image.shape  # Gets the size of the image
    center = width / 2  # Gets the center and the boundaries.
    left_boundary = width * .33
    right_boundary = width * .66
    top = height * .8
    for i in range(int(top)):
        for j in range(width):
            new_image[i][j] = [0, 0, 0]  # makes top of screen black

    image = new_image
    return image, center, left_boundary, right_boundary


def control_robot(robot, center_x, left_boundary, right_boundary):  # method to call the robot functions
    if center_x < left_boundary:
        robot.stop()
        robot.turn_left()

    elif center_x > right_boundary:
        robot.stop()
        robot.turn_right()

    else:
        robot.stop()
        robot.wheels_forward()


def initialize_cam():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.1)
    robot = robot_control.MoveRobot()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image, center, left_boundary, right_boundary = traverse_pixels(image)
        # Contours and thresholdhelped with sources https://stackoverflow.com/questions/114$
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        x, thresh = cv.threshold(image, 127, 255, 0)
        contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        moments = cv.moments(thresh)
        # center code source: https://www.learnopencv.com/find-center-of-blob-centroid-usin$

        center_x = int(moments["m10"] / moments["m00"])
        image = cv.drawContours(image, contours, -1, (150, 255, 255), 3)
        cv.imshow("Frame", image)

        control_robot(robot,center_x, left_boundary, right_boundary)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            break


def main():

    initialize_cam()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()


