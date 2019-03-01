import cv2 as cv
import numpy as np

original_image = None


def get_circles(image, lower, upper):
    kernel = np.ones((10, 10), np.uint8)  # Needed for erode and dilate functions
    new_image = cv.inRange(image, lower, upper)  # Shows colors in the color range set above
    new_image = cv.GaussianBlur(new_image, (11, 11), 0)  # Blurs the image in case of color range discrepancies.
    new_image = cv.erode(new_image, kernel, 1)  # Erodes the images that are in range
    new_image = cv.dilate(new_image, kernel, 1)  # Dilates the image in the color range

    # Circle code sourced from https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
    # circles = cv.HoughCircles(new_image, cv.HOUGH_GRADIENT, 5, 55, param1 = 800, param2=50, minRadius=10, maxRadius=30)  # Gets the circles from the image
    circles = cv.HoughCircles(new_image, cv.HOUGH_GRADIENT, 7, 55, param1=400, param2=50, minRadius=10,
                              maxRadius=30)  # Gets the circles from the image

    count = 0  # a counter that keeps track of the circles of the color range
    if circles is not None:  # Loops through the circles and marks the centers.  Marking the centers is likely not needed.  The loop is needed to count the colors
        circles = np.uint16(np.around(circles))
        count = 0
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            count += 1
            cv.circle(original_image, center, 1, (0, 100, 100), 3)

            cv.circle(new_image, center, 1, (0, 100, 100), 3)
            radius = circle[2]
            cv.circle(original_image, center, radius, (255, 0, 255), 3)

            cv.circle(new_image, center, radius, (255, 0, 255), 3)
    return new_image, count


def testing(img):
    green_lower = np.array([65, 0, 0])
    green_upper = np.array([84, 255, 255])
    blue_lower = np.array([85, 80, 40])
    blue_upper = np.array([166, 255, 255])
    yellow_lower = np.array([25, 0, 0])
    yellow_upper = np.array([30, 255, 255])
    orange_lower = np.array([0, 63, 215])
    orange_upper = np.array([21, 255, 255])
    brown_lower = np.array([60, 0, 52])
    brown_upper = np.array([133, 75, 156])
    red_lower = np.array([111, 104, 155])
    red_upper = np.array([180, 180, 246])

    image = cv.GaussianBlur(original_image, (5, 5), 50)  # Blurs the image
    image1 = cv.GaussianBlur(original_image, (15, 15), 150)  # Blurs the image

    v = np.median(image)
    lower1 = int(max(0, (1.0 - .33) * v))
    upper1 = int(min(255, (1.0 - .33) * v))

    v = np.median(image1)
    lower2 = int(max(0, (1.0 - .33) * v))
    upper2 = int(min(255, (1.0 - .33) * v))

    img1 = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    img2 = cv.cvtColor(image1, cv.COLOR_BGR2GRAY)
    test = cv.findContours(img1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    image = cv.Canny(img1, lower1, upper1)
    image1 = cv.Canny(img2, lower2, upper2)
    test = cv.compare(image, image1, 1)
    _, num = get_circles(image, lower1, upper1)
    cv.putText(test, "circles" + str(num), (200, 50), cv.FONT_HERSHEY_PLAIN, 2, [255, 255, 255])
    cv.imshow("wow", image)
    cv.imshow("wwwww", test)


def simplicity(img, channels, depth=3):
    blur = cv.GaussianBlur(img, (7, 7), cv.BORDER_DEFAULT)
    z = blur.reshape((-1, depth))
    z = np.float32(z)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret, label, center = cv.kmeans(z, channels, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)

    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape(img.shape)
    return res2


original_image = cv.imread("imagesWOvideo/candyBigSmallerTiny.jpg")
cv.imshow("orig", original_image)

blur = cv.GaussianBlur(original_image.copy(), (5, 5), cv.BORDER_DEFAULT)
hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

h, s, v = cv.split(hsv)
k = 4
h = simplicity(h, k, 1)
s = simplicity(s, k, 1)
v = simplicity(v, k, 1)
hsv_simple = [h, s, v]
image_simple = cv.cvtColor(cv.merge(hsv_simple), cv.COLOR_HSV2BGR)
test = cv.cvtColor(image_simple, cv.COLOR_RGB2GRAY)
cv.imshow("simplified", image_simple)

#testing(original_image)
# initial code helped from source: https://thecodacus.com/opencv-object-tracking-colour-detection-python/
image = cv.GaussianBlur(original_image, (7, 7), 0)  # Blurs the image
image = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image to HSV

# Bound values for colors
green_lower = np.array([65, 0, 0])
green_upper = np.array([84, 255, 255])
blue_lower = np.array([85, 80, 40])
blue_upper = np.array([166, 255, 255])
yellow_lower = np.array([25, 0, 0])
yellow_upper = np.array([30, 255, 255])
orange_lower = np.array([0, 63, 215])
orange_upper = np.array([21, 255, 255])
brown_lower = np.array([60, 0, 52])
brown_upper = np.array([133, 75, 156])
red_lower = np.array([111, 104, 155])
red_upper = np.array([180, 180, 246])

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
cv.imshow("Candy Image", brown_image)

cv.waitKey(0)
