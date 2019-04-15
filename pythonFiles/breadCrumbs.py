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
    top = height * .7
    new_image[0:int(top)][0:int(width)] = [0,0,0]
    return new_image, center, left_boundary, right_boundary


def control_robot(robot, center_x, left_boundary, right_boundary):  # method to call the robot functions
	if center_x < left_boundary:
		print("left")
		robot.stop()
		robot.turn_left()

	elif center_x > right_boundary:
		print("right")
		robot.stop()
		robot.turn_right()

	else:
		print("Straight")
		robot.stop()
		robot.wheels_forward()


def initialize_cam():
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(640, 480))
	time.sleep(0.1)
	robot = robot_control.MoveRobot()
	robot.center_robot()
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array)
		#        image = cv.Canny(image, 100, 170)
        	# show the frame
		new_image, center, left_boundary, right_boundary = traverse_pixels(image)
        # Contours and thresholdhelped with sources https://stackoverflow.com/questions/114$
		new_image = cv.cvtColor(new_image, cv.COLOR_BGR2GRAY)
		x, thresh = cv.threshold(new_image, 250, 255, 0)
		contours, h = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		moments = cv.moments(thresh)
		new_image = cv.drawContours(new_image, contours, -1, (150, 255, 255), 3)
		key = cv.waitKey(1) & 0xFF
		if moments["m00"] != 0:
			center_x = int(moments["m10"] / moments["m00"])
			control_robot(robot,center_x, left_boundary, right_boundary)

		cv.imshow("Frame", new_image)

        # clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		if key == ord("q"):
			break


def main():

    initialize_cam()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()


