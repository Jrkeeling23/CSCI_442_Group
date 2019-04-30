import numpy as np
import cv2 as cv

# "pink": [np.array([130, 130, 100]), np.array([190, 230, 235])
class Goal:

    def __init__(self, string_name):
        # Dictionaries for goals, lower thresh : upper thresh
        self.name = string_name
        self.goal = {"pink": [np.array([155, 105 , 55]), np.array([175, 195, 255])],
                     "green": [np.array([40, 155, 0]), np.array([60, 185, 255])]}
        self.current_x = 0
        self.current_y = 0

    def detect_ice(self, frame):
        """
        This function detects ice in frame by using blob detection.
        :param frame: Current frame.
        :return: True if there is a blob and false if there is not.
        """
        image = cv.GaussianBlur(frame.copy(), (5, 5), 0)
        roi = image[150:480, 100:600]
        hsv = cv.cvtColor(roi.copy(), cv.COLOR_BGR2HSV)

        thresholds = self.goal[self.name]  # get thresholds by name
        lower_thresh = thresholds[0]  # Thresholds are an array with lower to upper
        upper_thresh = thresholds[1]

        goal_mask = cv.inRange(hsv, lower_thresh, upper_thresh)  # works so far
        _, thresh = cv.threshold(goal_mask, 0, 250, cv.THRESH_BINARY_INV)  # convert between 0-250 to black
        # cv.imshow("roi", thresh)

        return self.blob_detector(thresh)

    def detect_bin(self, frame):
        """
        Detects the bin using blob detection. Will be called in the main/driver to determine where robot needs to move.
        :param frame: Current frame.
        :return: True if there is a blob and false if there is not.
        """
        image = cv.GaussianBlur(frame.copy(), (5, 5), 0)
        hsv = cv.cvtColor(image.copy(), cv.COLOR_BGR2HSV)

        lower_thresh = goal[0]
        upper_thresh = goal[lower_thresh]

        goal_mask = cv.inRange(hsv, lower_thresh, upper_thresh)  # works so far
        _, thresh = cv.threshold(goal_mask, 0, 250, cv.THRESH_BINARY_INV)  # convert between 0-250 to black

        return self.blob_detector(thresh)

    @staticmethod
    def blob_detector(frame):
        """
        Used by bin/ice detection. Gives parameters of blob detection and detects blobs.
        :param frame: Threshold image to detect.
        :return: True if there is a blob and false if there is not.
        """
        # detect blobs with these specifications
        params = cv.SimpleBlobDetector_Params()
        params.maxThreshold = 255
        params.minThreshold = 200
        params.filterByArea = False
        # params.minArea = 1500
        # params.maxArea = 50000
        params.filterByInertia = False
        params.filterByConvexity = False

        # detect the blobs (version differences require this to compile without error)
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv.SimpleBlobDetector(params)
        else:
            detector = cv.SimpleBlobDetector_create(params)
        keypoints = detector.detect(frame)
        img_with_keypoints = cv.drawKeypoints(thresh, keypoints, outImage=np.array([]), color=(0, 0, 255),
                                              flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints) is not 0:
            for i in range(len(keypoints)):
                x = keypoints[0].pt[0]
                y = keypoints[0].pt[1]

                # Lets us know if we are getting enough blobs in frame
                return True
        else:
            return False

    @staticmethod
    def line_detection(edge_frame, color_low, color_high):
        # detect blobs with these specifications
        params = cv.SimpleBlobDetector_Params()
        params.maxThreshold = 255
        params.minThreshold = 200
        params.filterByArea = True
        params.minArea = 1500
        params.maxArea = 50000
        params.filterByInertia = False
        params.filterByConvexity = False

        # detect the blobs (version differences require this to compile without error)
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv.SimpleBlobDetector(params)
        else:
            detector = cv.SimpleBlobDetector_create(params)

        keypoints = detector.detect(edge_frame)
        if len(keypoints) is not 0:
            for i in range(len(keypoints)):
                x = keypoints[0].pt[0]
                y = keypoints[0].pt[1]
                # Lets us know if we are getting enough blobs in frame
                return True
        else:
            return False

    def bin_area(self, frame):
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        thresholds = self.goal[self.name]  # get thresholds by name
        lower_thresh = thresholds[0]  # Thresholds are an array with lower to upper
        upper_thresh = thresholds[1]

        goal_mask = cv.inRange(hsv, lower_thresh, upper_thresh)  # works so far
        _, thresh = cv.threshold(goal_mask, 0, 250, cv.THRESH_BINARY)  # convert between 0-250 to black
        kernel = np.ones((5,5), np.uint8) 
        thresh = cv.erode(thresh,kernel, 20)
        thresh = cv.dilate(thresh, kernel, 5)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            cnt = contours[0]
            cv.imshow("", thresh)
            # print(cv.contourArea(cnt))
            # cv.drawContours(image, contours, -1, (0, 255, 0), 3)
            # when x is 0 (from top down is the axis), its left side of screen
            # when y is 0, its (from left to right), its top
            self.current_x = cnt[0][0][0]
            self.current_y = cnt[0][0][1]

            return True
        self.current_y = 0
        self.current_x = 0
        return False
