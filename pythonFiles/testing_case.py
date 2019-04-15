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
    left_boundary = width * .4
    right_boundary = width * .6
    top = int(height * .7)
    new_image[0:top][0:width] = [0,0,0]
    return new_image, center, left_boundary, right_boundary

def control_robot(robot, center_x, left_boundary, right_boundary):  # method to call the robot func$
	if center_x < left_boundary:
		print("left")
		#robot.turn_left()
	elif center_x > right_boundary:
		print("right")
		#robot.turn_right()
	else:
		print("Straight")
		#robot.wheels_forward()



def initialize_cam():
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(640,480 ))
	time.sleep(0.1)
	robot = robot_control.MoveRobot()
	robot.center_robot()
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		# image = cv.Canny(image, 100, 170)
		# show the frame
		#image, center, left_boundary, right_boundary = traverse_pixels(image)
		# Contours and thresholdhelped with sources https://stackoverflow.com/questions/114$
		image, center, left, right = traverse_pixels(image)

		gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		blur = cv.blur(gray.copy(), (3, 3))
		blur_color = cv.cvtColor(blur, cv.COLOR_GRAY2BGR)
		diff = cv.absdiff(blur_color, image)

		k1 = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
		k2 = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
		ret, mask = cv.threshold(diff, 10, 255, cv.THRESH_BINARY)
		diff = cv.bitwise_and(diff, mask)
		# diff = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
		diff = cv.inRange(diff, (10, 0, 0), (75, 50, 200))

		edge = cv.Canny(diff, 200, 255)
		edge = cv.dilate(edge, k2, iterations=1)
		edge = cv.erode(edge, k1, iterations=1)
		diff = cv.subtract(diff, edge)
		retval, threshold = cv.threshold(diff, 225, 255, cv.THRESH_BINARY_INV)

		contours, hierarchy = cv.findContours(threshold.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		cv.drawContours(threshold, contours, -1, (0,255,0),3)
		for c in contours:  # iterate through to find opposite corners for rectangle.
			if cv.contourArea(c) < 10000:
				continue
		(x, y, w, h) = cv.boundingRect(c)
		cv.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)  # draws rectangle within thresholds
		moments = cv.moments(threshold)
		if moments["m00"] != 0:
			center_x = int(moments["m10"] / moments ["m00"])
			control_robot(robot,center_x,left,right)
			
		cv.imshow("Frame", image)
		key = cv.waitKey(1) & 0xFF
		rawCapture.truncate(0)
		if key == ord("q"):
			break


def main():
	initialize_cam()
	cv.destroyAllWindows()


if __name__ == "__main__":
	main()
1














