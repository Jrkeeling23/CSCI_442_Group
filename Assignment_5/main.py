import cv2 as cv

image = cv.imread("square.png")
cv.namedWindow("test")
image = cv.Canny(image, 475, 200)
cv.imshow("test", image)

cv.waitKey(0)
