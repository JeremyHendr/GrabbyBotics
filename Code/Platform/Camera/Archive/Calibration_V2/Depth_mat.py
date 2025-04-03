import cv2 as cv
import numpy as np
import glob

images_names_right = glob.glob("Calibration_V2/pictures/depth_mat_test/Right/*")
images_names_left = glob.glob("Calibration_V2/pictures/depth_mat_test/Left/*")
draw_time = 3000
im_nb = 3
rightFrame = cv.imread(images_names_right[im_nb])
leftFrame = cv.imread(images_names_left[im_nb])
imageSize_frame = (rightFrame.shape[1], leftFrame.shape[0])

#Load rectification informations
calibration = np.load("Calibration_V2/retification_informations.npz", allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightROI = tuple(calibration["rightROI"])
rightMapY = calibration["rightMapY"]

assert imageSize_frame==imageSize, "Frame is not the same size as the calibration informations"

stereoMatcher = cv.StereoBM_create()


fixedLeft = cv.remap(leftFrame, leftMapX, leftMapY, cv.INTER_LINEAR )
fixedRight = cv.remap(rightFrame, rightMapX, rightMapY, cv.INTER_LINEAR)

# i = randint(1,1000)
i = 0
cv.imshow('left fixed', fixedLeft)
cv.imwrite("Calibration_V2/pictures/depth_mat_test/depth_map_and_rectify/"+str(i)+".jpg", fixedLeft)
cv.waitKey(draw_time)
cv.imshow('right fixed',fixedRight)
cv.imwrite("Calibration_V2/pictures/depth_mat_test/depth_map_and_rectify/"+str(i+1)+".jpg", fixedRight)
cv.waitKey(draw_time)

grayLeft = cv.cvtColor(fixedLeft, cv.COLOR_BGR2GRAY)
grayRight = cv.cvtColor(fixedRight, cv.COLOR_BGR2GRAY)
cv.imwrite("Calibration_V2/pictures/depth_mat_test/tune_BM/left.jpg",fixedLeft)
cv.imwrite("Calibration_V2/pictures/depth_mat_test/tune_BM/right.jpg",fixedRight)



#tune the stereo matcher
stereoMatcher.setMinDisparity(4)
stereoMatcher.setNumDisparities(128)
stereoMatcher.setBlockSize(21)
stereoMatcher.setSpeckleRange(16)
stereoMatcher.setSpeckleWindowSize(45)

depth = stereoMatcher.compute(grayLeft, grayRight)
DEPTH_VISUALIZATION_SCALE = 2048
cv.imwrite("Calibration_V2/pictures/depth_mat_test/depth_map_and_rectify/depth.jpg",depth)
cv.imshow('depth/SCALE', depth / DEPTH_VISUALIZATION_SCALE)
cv.waitKey(3000)



