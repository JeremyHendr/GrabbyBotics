import cv2 as cv
import glob
import numpy as np
from random import randint


def calibrate_camera(images_names, draw_duration=500):
    """
    Calibrate one camera by taking a sample of picture from it

    change de row/column number
    """
    images = []
    for image in images_names:
        im = cv.imread(image, 1)
        images.append(im)
  
    #criteria used by checkerboard pattern detector.
    #Change this if the code can't find the checkerboard
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    world_scaling = 1. #change this to the real world square size. Or not.
 
    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp
 
    #frame dimensions. Frames should be the same size.
    width = images[0].shape[1]
    height = images[0].shape[0]
 
    #Pixel coordinates of checkerboards
    imgpoints = [] # 2d points in image plane.
 
    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space
 
    for frame in images:
        #makes the frames gray
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        #find the checkerboard
        ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)

        assert ret, "No corners found in image"

        if ret == True:
            #Convolution size used to improve corner detection. Don't make this too large.
            conv_size = (11, 11)
 
            #opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
            cv.drawChessboardCorners(frame, (rows,columns), corners, ret)
            cv.imshow('image', frame)
            cv.waitKey(draw_duration)
 
            objpoints.append(objp)
            imgpoints.append(corners)
            
 
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (width, height), None, None)
    return mtx, dist



def stereo_calibrate(draw_duration, mtx1, dist1, mtx2, dist2, frames_names_c1, frames_names_c2):
    """
    Calibrate the two cameras together using the distortion parameter made by the singe calibration
    In this step we need to have the pictures from the two camera taken on the same time
    """
    #Reads images from camera 1 and 2
    c1_gray_frames = []
    c2_gray_frames = []

    for image_c1, image_c2 in zip(frames_names_c1, frames_names_c2):
        frame_c1 = cv.imread(image_c1, 1)
        frame_c2 = cv.imread(image_c2, 1)
        gray_frame_c1 = cv.cvtColor(frame_c1, cv.COLOR_BGR2GRAY)
        gray_frame_c2 = cv.cvtColor(frame_c2, cv.COLOR_BGR2GRAY)
        c1_gray_frames.append(gray_frame_c1)
        c2_gray_frames.append(gray_frame_c2)
  
    #change this if stereo calibration not good.
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    
    #change this to the real world square size. Or not.
    world_scaling = 1. 
 
    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp
 
    #frame dimensions. Frames should be the same size.
    width = c1_gray_frames[0].shape[1]
    height = c2_gray_frames[0].shape[0]
 
    #Pixel coordinates of checkerboards
    imgpoints_left = [] # 2d points in image plane.
    imgpoints_right = []
 
    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space
 
    for frame_c1, frame_c2 in zip(c1_gray_frames, c2_gray_frames):
        ret_c1, corners_c1 = cv.findChessboardCorners(frame_c1, (rows, columns), None)
        ret_c2, corners_c2 = cv.findChessboardCorners(frame_c2, (rows, columns), None)

        assert ret_c1 and ret_c2, "corners not found in one the two files"

        if ret_c1 and ret_c2:
            corners_c1 = cv.cornerSubPix(frame_c1, corners_c1, (11, 11), (-1, -1), criteria)
            corners_c2 = cv.cornerSubPix(frame_c2, corners_c2, (11, 11), (-1, -1), criteria)
 
            cv.drawChessboardCorners(frame_c1, (rows,columns), corners_c1, ret_c1)
            cv.imshow('img', frame_c1)
 
            cv.drawChessboardCorners(frame_c2, (rows,columns), corners_c2, ret_c2)
            cv.imshow('img2', frame_c2)
            cv.waitKey(draw_duration)
 
            objpoints.append(objp)
            imgpoints_right.append(corners_c1)
            imgpoints_left.append(corners_c2)
 
    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_right, imgpoints_left, mtx1, dist1,
                                                                 mtx2, dist2, (width, height), criteria = criteria, flags = stereocalibration_flags)
 
    return R, T





#------------------------------------------------------------------------------------------------------------------------
#Inner size of the chessboard
rows = 9 #number of checkerboard rows.
columns = 9 #number of checkerboard columns.

#Grabbing images
#images downloaded
# images_names_c1 = glob.glob("Calibration_V2/Chessboard_pictures/Right/*.png")
# images_names_c2 = glob.glob("Calibration_V2/Chessboard_pictures/Left/*.png")

#My images
images_names_c1 = glob.glob("Calibration_V2/pictures/Right/*")
images_names_c2 = glob.glob("Calibration_V2/pictures/Left/*")

assert len(images_names_c1) > 0 and len(images_names_c2) > 0, "No images found in one the Files"

#verifying dimension of images
# leftFrame = cv.imread(images_names_c1[0])
# rightFrame = cv.imread(images_names_c2[0])
# imageSize = (rightFrame.shape[0], leftFrame.shape[1])
# print(imageSize)

matrix_c1, distortion_coef_c1 = calibrate_camera(images_names_c1,10)
matrix_c2, distortion_coef_c2 = calibrate_camera(images_names_c2,10)

print("Distortion coefficient:\nR:\n"+str(distortion_coef_c1)+"\nL:\n"+str(distortion_coef_c2))
print("matrix:\nR:\n"+str(matrix_c1)+"\nL:\n"+str(matrix_c2))

rotation_matrix, translation_vector = stereo_calibrate(10, matrix_c1, distortion_coef_c1, matrix_c2, distortion_coef_c2, images_names_c1, images_names_c2)

print("\nRotation matrix:\n"+str(rotation_matrix))
print("\ntranslation vector\n"+str(translation_vector))

rightFrame = cv.imread(images_names_c1[0])
leftFrame = cv.imread(images_names_c2[0])
imageSize = (rightFrame.shape[1], leftFrame.shape[0])

print("\nImages used for depth mat:\n"+images_names_c1[0]+"\n"+images_names_c2[0])
print(leftFrame.shape, rightFrame.shape)

#imageSize=(w,h)
rightRectification, leftRectification, rightProjection, leftProjection, dispartityToDepthMap, rightROI, leftROI = cv.stereoRectify(
                matrix_c1, distortion_coef_c1,
                matrix_c2, distortion_coef_c2,
                imageSize, 
                rotation_matrix, translation_vector,
                None, None, None, None, None,
                cv.CALIB_ZERO_DISPARITY)
#LEFT
leftMapX, leftMapY = cv.initUndistortRectifyMap(matrix_c2, distortion_coef_c2, 
                                                leftRectification, leftProjection,
                                                imageSize, cv.CV_32FC1)
#RIGHT
rightMapX, rightMapY = cv.initUndistortRectifyMap(matrix_c1, distortion_coef_c1, 
                                                  rightRectification, rightProjection, 
                                                  imageSize, cv.CV_32FC1)

#Save the calibration information to further use
np.savez_compressed("Calibration_V2/retification_informations",
                    imageSize=imageSize,
                    leftMapX=leftMapX,   leftMapY=leftMapY, 
                    rightMapX=rightMapX, rightMapY=rightMapY,
                    leftROI=leftROI,     rightROI=rightROI)


"""
leftRectification, rightRectification, leftProjection, rightProjection, dispartityToDepthMap, leftROI, rightROI = cv.stereoRectify(
                matrix_c1, 
                distortion_coef_c1,
                matrix_c2, 
                distortion_coef_c2,
                imageSize, 
                rotation_matrix, 
                translation_vector,
                None, None, None, None, None,
                cv.CALIB_ZERO_DISPARITY)"""




# stereoMatcher = cv.StereoBM_create()

# fixedLeft = cv.remap(leftFrame, leftMapX, leftMapY, cv.INTER_LINEAR )
# fixedRight = cv.remap(rightFrame, rightMapX, rightMapY, cv.INTER_LINEAR)

# # i = randint(1,1000)
# i = 0
# cv.imshow('left fixed', fixedLeft)
# cv.imwrite("Calibration_V2/pictures/Distortion_fixed/"+str(i)+".jpg", fixedLeft)
# cv.waitKey(3000)
# cv.imshow('right fixed',fixedRight)
# cv.imwrite("Calibration_V2/pictures/Distortion_fixed/"+str(i+1)+".jpg", fixedRight)
# cv.waitKey(3000)


# """
# grayLeft = cv.cvtColor(fixedLeft, cv.COLOR_BGR2GRAY)
# grayRight = cv.cvtColor(fixedRight, cv.COLOR_BGR2GRAY)
# depth = stereoMatcher.compute(grayLeft, grayRight)

# DEPTH_VISUALIZATION_SCALE = 2048
# cv.imshow('depth', depth / DEPTH_VISUALIZATION_SCALE)
# cv.waitKey(3000)
# """
