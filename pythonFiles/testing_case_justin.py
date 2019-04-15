import robot_control
import cv2 as cv
import numpy as np
from picamera import PiCamera
import time
from picamera.array import PiRGBArray

def initialize_cam():
	camera = PiCamera()
	camera.resolution = (540, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(640, 480))
	time.sleep(0.1)
#	robot = robot_control.MoveRobot()
#	robot.center_robot()
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		w, h = image.shape[:2]
		x,y = w // 5, h //2
		image = image[y:y+h//4, x:x + 3*w//5]
		#detect_line(image)
		# image = cv.Canny(image, 100, 170)
		# show the frame
		#image, center, left_boundary, right_boundary = traverse_pixels(image)
		# Contours and thresholdhelped with sources https://stackoverflow.com/questions
		gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		blur = cv.blur(gray,(3,3))
		blur_color = cv.cvtColor(blur, cv.COLOR_GRAY2BGR)
		sub = cv.subtract(image, blur_color)
		#hsv = cv.cvtColor(sub, cv.COLOR_BGR2HSV)
		#edges = cv.Canny(sub, 100, 170)
		lower_red = np.array([0,0,60])
		upper_red = np.array([0,0,255])
		mask = cv.inRange(sub,lower_red, upper_red)
		#edges = cv.Canny(sub, 50,170)
		res = cv.bitwise_xor(sub, image, mask=mask)
		#cv.normalize(res, res, 0 ,60)

		thresh, image_thresh = cv.threshold(image, 127,255,0)
		flood_fill = image_thresh.copy()
		h, w =  image_thresh.shape[:2]
		mask = np.zeros((h+2, w+2), np.uint8)
		cv.floodFill(flood_fill, mask, (0,0), 65)
		#contours, h = cv.findContours(flood_fill, cv.RETR_FLOODFILL, cv.CHAIN_APPROX_SIMPLE)
		#moments = cv.moments(flood_fill)
		#image.drawContours(image, contours, -1, (150, 255,255), 3)
		#centerX = int(moments["m10"]/moments["m00"])
		##flood_fill_inv = cv.bitwise_not(flood_fill)
		##image_out = image_thresh | flood_fill_inv
		edges = cv.Canny(flood_fill, 247,255)
		cv.imshow("Frame", edges)
		key = cv.waitKey(1) & 0xFF
		rawCapture.truncate(0)
		if key == ord("q"):
			break

def detect_line(image):
	edges = np.zeros(image.shape, np.uint8)
	edges = cv.GaussianBlur(edges, (9,9),cv.BORDER_DEFAULT)
	cv.normalize(image, edges, 0 ,255, cv.NORM_MINMAX)
	edges = cv.Canny(edges, 247, 255)
	edges = cv.dilate(edges, np.ones((2,2)))
	cv.imshow("Frame", edges)

def main():
	initialize_cam()
	detect_line()
	cv.destroyAllWindows()


if __name__ == "__main__":
	main()















