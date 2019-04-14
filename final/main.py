import cv2 as cv
import numpy as np


class ImageManipulation:

    def edge_detection(self, img):  # Method to get the edges
        # Code sourced for canny from https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
        # image = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img = cv.GaussianBlur(img, (3, 3), 0)  # Blurs the image to smooth it out.
        #
        # kernel = np.ones((3, 11), np.float32)
        # image = cv.filter2D(img, -1, kernel)  # Blurs the image
        v = np.median(img)
        sigma = 0.33
        low = int(max(0, (1.0 - sigma) * v))
        high = int(min(255, (1.0 + sigma) * v))
        image = cv.Canny(img, low, high)
        # image = img.copy()
        # # sourced from https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
        # # kernel = np.ones((3, 3), np.float32) / 25
        # image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # # image = cv.filter2D(image, -1, kernel)  # Blurs the image
        # image = cv.GaussianBlur(image, (3,3), 0) # Blurs the image to smooth it out.
        #
        # image = cv.Canny(image, 20, 120) # Creates edges

        return image

    def fill_image(self, image):
        # Code sourced for this method from https://stackoverflow.com/questions/45135950/how-to-fill-an-image-from-bottom-side-until-an-edge-is-detected-using-opencv
        height, width = image.shape[:2]
        max_row_index = height - np.argmax(image[::-1], axis=0)  # Inverts the edges from top to bottom
        row_index = np.indices((height, width))[0]
        after_edge = row_index >= max_row_index  # Sets edges to fill to
        filled_image = np.zeros((height, width))
        filled_image[after_edge] = 255  # fills the image
        # erode code sourced from https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        kernel = np.ones((5, 5), np.uint8)
        filled_image = cv.erode(filled_image, kernel,
                                iterations=7)  # Erodes the image to remove small unusable lines for the robot

        filled_image = np.array(filled_image, dtype=np.uint8)
        filled_image = cv.merge((filled_image, filled_image,
                                 filled_image))  # converts the image back from 1 channel to 3 channel sourced from https://stackoverflow.com/questions/14786179/how-to-convert-a-1-channel-image-into-a-3-channel-with-opencv2

        # ret, bw_thresh = cv.threshold(filled_image, 1, 2550, cv.THRESH_BINARY)
        return filled_image

    def smooth(self, image):
        # Idea for medianblur sourced from https://pythonprogramming.net/blurring-smoothing-python-opencv-tutorial/
        image = cv.medianBlur(image, 15)
        return image

    def getHighestCoordinate(self, image):
        try:
            pixels = np.where(image == [255, 255, 255])
            y_length = len(pixels[0])
            max_white_value = np.max(pixels[0])
            if max_white_value < y_length:
                y_val = pixels[0][max_white_value]
                x_val = pixels[1][max_white_value]
                image = cv.circle(image, (x_val, y_val), 1, (0, 0, 255), 10)
        except ValueError:
            pass
        return image


cap = cv.VideoCapture(0)

while True:
    status, img = cap.read()

    # img = cv.imread('im2.jpg')
    cv.imshow("original", img)
    manipulation = ImageManipulation()
    image = manipulation.edge_detection(img.copy())
    image = manipulation.fill_image(image)
    image = manipulation.smooth(image)
    image = manipulation.getHighestCoordinate(image)
    cv.imshow("Video", image)
    k = cv.waitKey(1)
    if k == 27:
        break
cv.destroyAllWindows()
