# Alex Harry
# Justin Keeling
import cv2 as cv
import numpy as np

original_image = None
def get_circles(image, lower, upper):
    height, width, x = image.shape # Gets the size of the image
    kernel = np.ones((10, 10), np.uint8)  # Needed for erode and dilate functions
    new_image = cv.inRange(image, lower, upper)  # Shows colors in the color range set above
    new_image = cv.GaussianBlur(new_image, (7, 7), 0)  # Blurs the image in case of color range discrepancies.
    new_image = cv.erode(new_image, kernel, 1)  # Erodes the images that are in range
    new_image = cv.dilate(new_image, kernel, 10) # Dilates the image in the color range

    # Circle code sourced from https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
    image_size = height * width
    if image_size < 480001:
        circles = cv.HoughCircles(new_image, cv.HOUGH_GRADIENT, 5, 8, param1 = 136, param2=45, minRadius=9, maxRadius=19)  # Gets the circles from the image
    elif image_size < 853601:
        circles = cv.HoughCircles(new_image, cv.HOUGH_GRADIENT, 5, 30, param1 = 136, param2=45, minRadius=9, maxRadius=19)  # Gets the circles from the image
    elif image_size < 1920001:
        circles = cv.HoughCircles(new_image, cv.HOUGH_GRADIENT, 5, 30, param1 = 136, param2=45, minRadius=20, maxRadius=25)  # Gets the circles from the image

    # Circle code sourced from https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
    count = 0 # a counter that keeps track of the circles of the color range
    if circles is not None: # Loops through the circles and marks the centers.  Marking the centers is likely not needed.  The loop is needed to count the colors
        circles = np.uint16(np.around(circles))
        count = 0
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            count += 1
            cv.circle(original_image, center, 1,  (0,100,100),3) # Marks the center of the circle
            cv.circle(new_image, center, 1,  (0,100,100),3)
            radius = circle[2]
            cv.circle(original_image, center, radius, (255,0,255), 3) # Draws the radius
            cv.circle(new_image, center, radius, (255,0,255), 3)
    return new_image, count

original_image = cv.imread("stitchedImages/candyBigSmallerTiny.jpg")
cv.imshow("orig", original_image)
# initial code helped from source: https://thecodacus.com/opencv-object-tracking-colour-detection-python/
image = cv.GaussianBlur(original_image, (7, 7), 0)  # Blurs the image
image = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image to HSV

# Bound values for colors
green_lower = np.array([65, 0, 0])
green_upper = np.array([84, 255, 255])
blue_lower = np.array([85, 80, 40])
blue_upper = np.array([110, 255, 255])
yellow_lower = np.array([23, 0, 0])
yellow_upper = np.array([30, 255, 255])
orange_lower = np.array([0, 63, 215])
orange_upper = np.array([21, 255, 255])
brown_lower = np.array([60, 0, 52])
brown_upper = np.array([111, 75, 156])
red_lower = np.array([111, 104, 155])
red_upper = np.array([180, 180, 246])




# Runs the method for getting the circles and circle count for given color range.
green_image, num_green = get_circles(image, green_lower, green_upper)
blue_image, num_blue = get_circles(image, blue_lower, blue_upper)
yellow_image, num_yellow = get_circles(image, yellow_lower, yellow_upper)
orange_image, num_orange = get_circles(image, orange_lower, orange_upper)
red_image, num_red = get_circles(image, red_lower, red_upper)

# The following adds text to the original image.
cv.putText(original_image, "Blue: " + str(num_blue), (10, 50), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Green: " + str(num_green), (200, 50), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Yellow: " + str(num_yellow), (10, 100), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Orange: " + str(num_orange), (200, 100), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
cv.putText(original_image, "Red: " + str(num_red), (200, 150), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])


# Shows images.
cv.imshow("Candy Image original", original_image)

cv.waitKey(0)