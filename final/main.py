import cv2 as cv
import imageManipulation
import makeMoves


class Driver:

    def __init__(self):
        self.manipulation = imageManipulation.ImageManipulation()
        self.cap = cv.VideoCapture(0)
        status, img = self.cap.read()
        #img = cv.imread('im2.jpg')

        self.height, self.width, _ = img.shape
        self.move = makeMoves.Move(self.width, self.height)

    def run(self):
        while True:
            status, img = self.cap.read()
            #img = cv.imread('im2.jpg')
            image = self.manipulation.edge_detection(img.copy())
            image = self.manipulation.fill_image(image)
            image = self.manipulation.smooth(image)
            print
            image, x_coordinate, y_coordinate = self.manipulation.getHighestCoordinate(image, int(self.width / 2), self.height)
            # self.move.decide_move(x_coordinate, y_coordinate)
            overlayed = cv.addWeighted(img, .7, image, 0.4, 0)  # Overlays the path on the original image
            cv.imshow("Paths", overlayed)

            k = cv.waitKey(1)
            if k == 27:
                break
        cv.destroyAllWindows()


driver = Driver()
driver.run()
