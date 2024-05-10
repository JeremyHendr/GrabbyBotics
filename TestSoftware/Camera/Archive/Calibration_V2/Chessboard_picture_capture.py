import cv2 as cv
import numpy as np

#size of the image
imageSize = (width, height) 

def calibrate_camera():
    pass
def stereo_calibrate():
    pass

#----------------------
#TODO: check if c1 is left or right
matrix_c1, distortion_coef_c1 = calibrate_camera(images_names_c1,100)
matrix_c2, distortion_coef_c2 = calibrate_camera(images_names_c2,100)

rotation_matrix, translation_vector = stereo_calibrate(mtx1, dist1, mtx2, dist2, images_names_c1, images_names_c2)


leftRectification, rightRectification, leftProjection, rightProjection, dispartityToDepthMap, leftROI, rightROI = cv.stereoRectify(
                matrix_c1, 
                distortion_coef_c1,
                matrix_c2, 
                distortion_coef_c2,
                imageSize, 
                rotation_matrix, 
                translation_vector,
                None, None, None, None, None,
                cv.CALIB_ZERO_DISPARITY)

#LEFT
leftMapX, leftMapY = cv.initUndistortRectifyMap(matrix_c1, 
                                                distortion_coef_c1, 
                                                leftRectification,
                                                leftProjection,
                                                imageSize, 
                                                cv.CV_32FC1)

#RIGHT
rightMapX, rightMapY = cv.initUndistortRectifyMap(matrix_c2, 
                                                  distortion_coef_c2, 
                                                  rightRectification,
                                                  rightProjection, 
                                                  imageSize, cv.CV_32FC1)

#save the results of the calibration
np.savez_compressed("outputFile.npy",
                    imageSize=imageSize,
                    leftMapX=leftMapX, 
                    leftMapY=leftMapY, 
                    rightMapX=rightMapX,
                    leftROI=leftROI,
                    rightMapY=rightMapY, 
                    rightROI=rightROI)


#----------------------other file
#Load the saved calibration
calibration = np.load("outputFile.npy", allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightROI = tuple(calibration["rightROI"])
rightMapY = calibration["rightMapY"]

stereoMatcher = cv.StereoBM_create()
# TODO: stereo matcher can be tuned fro bether quality images
# UI to tune the parameters
# https://erget.wordpress.com/2014/03/13/building-an-interactive-gui-with-opencv/
# parameters from someone
"""
stereoMatcher.setMinDisparity(4)
stereoMatcher.setNumDisparities(128)
stereoMatcher.setBlockSize(21)
stereoMatcher.setSpeckleRange(16)
stereoMatcher.setSpeckleWindowSize(45)
"""


#TODO read the left and right frame

fixedLeft = cv.remap(leftFrame, leftMapX, leftMapY, cv.INTER_LINEAR )
fixedRight = cv.remap(rightFrame, rightMapX, rightMapY, cv.INTER_LINEAR)

grayLeft = cv.cvtColor(fixedLeft, cv.COLOR_BGR2GRAY)
grayRight = cv.cvtColor(fixedRight, cv.COLOR_BGR2GRAY)
depth = stereoMatcher.compute(grayLeft, grayRight)

DEPTH_VISUALIZATION_SCALE = 2048
cv.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)