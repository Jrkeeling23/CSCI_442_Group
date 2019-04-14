import cv2 as cv
import numpy as np


class ImageManipulation:

    def edge_detection(self, img): # Method to get the edges
        image = img.copy()
        # sourced from https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
        kernel = np.ones((3, 5), np.float32) / 25
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image = cv.filter2D(image, -1, kernel)  # Blurs the image
        image = cv.Canny(image, 50, 200, True) # Creates edges
        return image


    def fill_image(self, image):
# Code sourced for this method from https://stackoverflow.com/questions/45135950/how-to-fill-an-image-from-bottom-side-until-an-edge-is-detected-using-opencv
        height, width = image.shape[:2]
        max_row_index = height - np.argmax(image[::-1], axis=0) # Inverts the edges from top to bottom
        row_index = np.indices((height,width))[0]
        after_edge = row_index >= max_row_index # Sets edges to fill to
        filled_image = np.zeros((height, width))
        filled_image[after_edge] = 255  # fills the image
        return filled_image
cap = cv.VideoCapture(0)

while True:
    status, img = cap.read()
    manipulation = ImageManipulation()
    image = manipulation.edge_detection(img.copy())
    image = manipulation.fill_image(image)
    cv.imshow("Video", image)
    k = cv.waitKey(1)
    if k == 27:
        break
cv.destroyAllWindows()
