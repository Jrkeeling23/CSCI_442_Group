import cv2 as cv
import numpy as np


def get_image_range(image, lower, upper):
    new_image = cv.inRange(image, lower, upper)  # Shows colors in the color range set above
    new_image = cv.GaussianBlur(new_image, (7, 7), 0)  # Blurs the image in case of color range discrepancies.
    return new_image


def get_keypoints(image):
    blob_detection = cv.SimpleBlobDetector_create()  # Creates the blob detector
    return blob_detection.detect(
        cv.bitwise_not(image))  # Blob detection doesn't work with a black background.  bitwise_not fixes this.


original_image = cv.imread("imagesWOvideo/one.jpg")
# initial code helped from source: https://thecodacus.com/opencv-object-tracking-colour-detection-python/
image = cv.GaussianBlur(original_image, (7, 7), 0)  # Blurs the image
image = cv.cvtColor(image, cv.COLOR_BGR2HSV)  # Converts the image to HSV

# Bound values for colors
green_lower = np.array([33, 80, 40])
green_upper = np.array([102, 255, 255])
blue_lower = np.array([198, 79, 88])
blue_upper = np.array([163, 284, 78])
yellow_lower = np.array([58, 78, 87])
yellow_upper = np.array([163, 284, 78])
orange_lower = np.array([13, 59, 85])
orange_upper = np.array([163, 284, 78])
brown_lower = np.array([22, 35, 30])
brown_upper = np.array([163, 284, 78])

green_image = get_image_range(image, green_lower, green_upper)

num_green = get_keypoints(green_image)

print(len(num_green))
cv.putText(original_image, "Green: " + str(len(num_green)), (10,50), cv.FONT_HERSHEY_PLAIN, 2, [255,255,255])
cv.imshow("Candy Image", green_image)
cv.imshow("Candy Image original", original_image)

cv.waitKey(0)
