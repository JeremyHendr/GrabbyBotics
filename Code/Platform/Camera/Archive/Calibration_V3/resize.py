import cv2 as cv
import numpy as np


cv.namedWindow('Camera',cv.WINDOW_AUTOSIZE)

#test images
left = cv.imread("Test_pictures/Normal/0_L.jpg")
right = cv.imread("Test_pictures/Normal/0_R.jpg")
print(left)

scale = 0.4
left_resized = cv.resize(left, None, fx = scale, fy = scale)
right_resized = cv.resize(right, None, fx = scale, fy = scale)

stack = np.hstack((left_resized,right_resized))

cv.imshow("Camera",stack)
# cv.imshow("normal",left)
cv.waitKey(3000)
while True:
    pass


cv.destroyAllWindows()
