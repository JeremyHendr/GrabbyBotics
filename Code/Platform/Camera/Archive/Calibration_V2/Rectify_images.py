import cv2 as cv
import numpy as np

rightFrame = cv.imread("Calibration_V2/pictures/Right/0_R.jpg")
leftFrame = cv.imread("Calibration_V2/pictures/left/0_L.jpg")
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
cv.imwrite("Calibration_V2/pictures/Distortion_fixed/"+str(i)+".jpg", fixedLeft)
cv.waitKey(3000)
cv.imshow('right fixed',fixedRight)
cv.imwrite("Calibration_V2/pictures/Distortion_fixed/"+str(i+1)+".jpg", fixedRight)
cv.waitKey(3000)

grayLeft = cv.cvtColor(fixedLeft, cv.COLOR_BGR2GRAY)
grayRight = cv.cvtColor(fixedRight, cv.COLOR_BGR2GRAY)
depth = stereoMatcher.compute(grayLeft, grayRight)

DEPTH_VISUALIZATION_SCALE = 2048
cv.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)
cv.waitKey(3000)
