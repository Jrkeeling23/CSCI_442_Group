import cv2 as cv
import numpy as np


def get_circles(image, lower, upper):
    kernel = np.ones((10, 10), np.uint8)  # Needed for erode and dilate functions
    new_image = cv.inRange(image, lower, upper)  # Shows colors in the color range set above
    new_image = cv.GaussianBlur(new_image, (5, 5), 0)  # Blurs the image in case of color range discrepancies.
    new_image = cv.erode(new_image, kernel, 1)  # Erodes the images that are in range
    new_image = cv.dilate(new_image, kernel, 1) # Dilates the image in the color range

    # Circle code sourced from https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
    circles = cv.HoughCircles(new_image, cv.HOUGH_GRADIENT, 5, 55, param1 = 800, param2=50, minRadius=10, maxRadius=30)  # Gets the circles from the image
    count = 0 # a counter that keeps track of the circles of the color range
    if circles is not None: # Loops through the circles and marks the centers.  Marking the centers is likely not needed.  The loop is needed to count the colors
        circles = np.uint16(np.around(circles))
        count = 0
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            count += 1
            cv.circle(new_image, center, 1,  (0,100,100),3)
            radius = circle[2]
            cv.circle(new_image, center, radius, (255,0,255), 3)
    return new_image, count 

original_image = cv.imread("imagesWOvideo/four.jpg")
# initial code helped from source: https://thecodacus.com/opencv-object-tracking-colour-detection-python/
image = cv.GaussianBlur(original_image, (7, 7), 0)  # Blurs the image
image = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image to HSV

# Bound values for colors
green_lower = np.array([65, 0, 0])
green_upper = np.array([84, 255, 255])
blue_lower = np.array([84, 80, 40])
blue_upper = np.array([166, 255, 255])
yellow_lower = np.array([25, 0, 0])
yellow_upper = np.array([30, 255, 255])
orange_lower = np.array([20, 0, 0])
orange_upper = np.array([30, 255, 255])
brown_lower = np.array([10, 0, 0])
brown_upper = np.array([20, 255, 255])
red_lower = np.array([0, 0, 0])
red_upper = np.array([118, 255, 255])

# Runs the method for getting the circles and circle count for given color range.
green_image, num_green = get_circles(image, green_lower, green_upper)
blue_image, num_blue = get_circles(image, blue_lower, blue_upper)
yellow_image, num_yellow = get_circles(image, yellow_lower, yellow_upper)
orange_image, num_orange = get_circles(image, orange_lower, orange_upper)
brown_image, num_brown = get_circles(image, brown_lower, brown_upper)
red_image, num_red = get_circles(image, red_lower, red_upper)

# The following adds text to the original image.
cv.putText(original_image, "Blue: " + str(num_blue), (10, 50), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Green: " + str(num_green), (200, 50), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Yellow: " + str(num_yellow), (10, 100), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Brown: " + str(num_brown), (200, 100), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Orange: " + str(num_orange), (10, 150), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Red: " + str(num_red), (200, 150), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])


# Shows images.
cv.imshow("Candy Image original", original_image)
# cv.imshow("Candy Image", orange_image)

cv.waitKey(0)
