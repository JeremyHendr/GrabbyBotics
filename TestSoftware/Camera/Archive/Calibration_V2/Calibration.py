import cv2 as cv
import glob
import numpy as np


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



def stereo_calibrate(mtx1, dist1, mtx2, dist2, frames_names_c1, frames_names_c2):
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
 
            cv.drawChessboardCorners(frame_c1, (5,8), corners_c1, ret_c1)
            cv.imshow('img', frame_c1)
 
            cv.drawChessboardCorners(frame_c2, (5,8), corners_c2, ret_c2)
            cv.imshow('img2', frame_c2)
            cv.waitKey(500)
 
            objpoints.append(objp)
            imgpoints_left.append(corners_c1)
            imgpoints_right.append(corners_c2)
 
    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx1, dist1,
                                                                 mtx2, dist2, (width, height), criteria = criteria, flags = stereocalibration_flags)
 
    return R, T
 
#------------------------------------------------------------------------------------------------------------------------
#Inner size of the chessboard
rows = 4 #number of checkerboard rows.
columns = 7 #number of checkerboard columns.

#Grabbing images
images_names_c1 = glob.glob("Calibration_V2/Chessboard_pictures/Right/*.png")
images_names_c2 = glob.glob("Calibration_V2/Chessboard_pictures/Left/*.png")

#TODO: assert images in liste

mtx1, dist1 = calibrate_camera(images_names_c1,100)
mtx2, dist2 = calibrate_camera(images_names_c2,100)

R, T = stereo_calibrate(mtx1, dist1, mtx2, dist2, images_names_c1, images_names_c2)
 
#this call might cause segmentation fault error. This is due to calling cv.imshow() and plt.show()
triangulate(mtx1, mtx2, R, T)