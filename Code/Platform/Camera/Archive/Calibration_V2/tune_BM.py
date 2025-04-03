import cv2 as cv
import numpy as np


fixedLeft = cv.imread("Calibration_V2/pictures/depth_mat_test/tune_BM/left.jpg")
fixedRight = cv.imread("Calibration_V2/pictures/depth_mat_test/tune_BM/right.jpg")
grayLeft = cv.cvtColor(fixedLeft, cv.COLOR_BGR2GRAY)
grayRight = cv.cvtColor(fixedRight, cv.COLOR_BGR2GRAY)


stereoMatcher = cv.StereoBM_create()


# #tune the stereo matcher
# stereoMatcher.setMinDisparity(4)
# stereoMatcher.setNumDisparities(128)
# stereoMatcher.setBlockSize(21)
# stereoMatcher.setSpeckleRange(16)
# stereoMatcher.setSpeckleWindowSize(45)

DEPTH_VISUALIZATION_SCALE = 2048

depth = stereoMatcher.compute(grayLeft, grayRight)

depth = (depth).astype(np.uint8)
depth = cv.applyColorMap(depth, cv.COLORMAP_JET)

cv.imwrite("Calibration_V2/pictures/depth_mat_test/tune_BM/depth.jpg",depth)
cv.imshow('depth/SCALE', depth / DEPTH_VISUALIZATION_SCALE)
cv.waitKey(3000)
