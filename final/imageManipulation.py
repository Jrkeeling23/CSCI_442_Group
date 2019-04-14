import cv2 as cv
import numpy as np


class ImageManipulation:

    def edge_detection(self, img):  # Method to get the edges
        # Code sourced for canny from https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
        # Code optimizes edges without needed to manually adjust params
        img = cv.GaussianBlur(img, (3, 3), 0)  # Blurs the image to smooth it out.
        v = np.median(img)
        sigma = 0.33
        low = int(max(0, (1.0 - sigma) * v))
        high = int(min(255, (1.0 + sigma) * v))
        image = cv.Canny(img, low, high)
        return image

    def fill_image(self, image): # Fills the image where robot can move
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
        return filled_image

    def smooth(self, image): # Smooths the edges of the imave
        # Idea for medianblur sourced from https://pythonprogramming.net/blurring-smoothing-python-opencv-tutorial/
        image = cv.medianBlur(image, 15)
        return image

    def getHighestCoordinate(self, image, center, height): # gets the highest coordinate that the robot will try to move to
        x_val = None
        y_val = None
        try:
            pixels = np.where(image == [255, 255, 255]) # finds all pixels of white value
            y_length = len(pixels[0])   # length of y array for future checking
            max_white_value = np.max(pixels[0]) # gets the max y value of white pixel
            if max_white_value < y_length:
                y_val = pixels[0][max_white_value]
                x_val = pixels[1][max_white_value]
                image = cv.circle(image, (x_val, y_val), 2, (0, 0, 255), 10) # draws a circle on the highest white pixel
                image = self.draw_rectangle(image)
                cv.putText(image, 'Highest Middle X: ' + str(x_val), (10, 30), cv.FONT_HERSHEY_DUPLEX, 1,
                           (0, 0, 0), 1)
                cv.line(image, (center, height), (x_val, y_val + 8), (0,0,255), 3)# Draws a line from center of robot to intended destination point

        except ValueError:
            pass
        return image, x_val, y_val

    def draw_rectangle(self, image): # Draws rectangle for text box
        image = cv.rectangle(image, (3,4), (370,42), (0,0,255), 2) # Rectangle border
        image = cv.rectangle(image, (4,5), (369,41), (255,255,255), cv.FILLED, 8) #fills rectangle
        return image

