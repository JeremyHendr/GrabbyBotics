"""
This script determines the camera parameters to be able to rectify the frames.

There is no need to run this script multiple times, one is enough to get the right parameters.
The images used for the calibration should be taken simultaneously by the two cameras.
4 pictures is the minimum more is bether, also see the function description to take your picture correctly.
Make sur to change the rows and columns numbers

The results will be saved in a .npz file.
"""

import cv2 as cv
import glob
import numpy as np


def __main__():
    """Created only for convenience"""

    #NOTE: The following parameters have to be modified to suit your needs
    # INNER size of the chessboard
    rows = 9
    columns = 9
    ##--------------------------------------------------------------------

    #Images taken SIMULTANEOUSLY by the two cameras
    #left camera is when we look from the point of view of the cameras (if look at the cameras it will be the one to your right)
    images_names_right = sorted(glob.glob("Calibration_pictures/Right/*"))
    images_names_left = sorted(glob.glob("Calibration_pictures/Left/*"))

    assert len(images_names_right) > 0 and len(images_names_left) > 0, "No images found in one the Files"

    #getting the size of the images (they should all be the same size)
    frame = cv.imread(images_names_right[0])
    imageSize = (frame.shape[1], frame.shape[0])

    #getting the camera matrix and distortion coefficients for both cameras
    matrix_right, distortion_coef_right = calibrate_camera(images_names_right, rows, columns, 1)
    print("right camera calibrated")
    matrix_left, distortion_coef_left = calibrate_camera(images_names_left, rows, columns, 1)
    print("left camera calibrated")

    print("Distortion coefficient:\nRight: "+str(distortion_coef_right)+"\nLeft: "+str(distortion_coef_left))
    print("Camera matrix:\nRight:\n"+str(matrix_right)+"\nLeft:\n"+str(matrix_left))

    #Getting the rotation matrix and translation vector to go from one camera to the other
    rotation_matrix, translation_vector = stereo_calibrate(matrix_right, distortion_coef_right, matrix_left, distortion_coef_left, images_names_right, images_names_left, rows, columns)

    print("\nRotation matrix:\n"+str(rotation_matrix))
    print("\ntranslation vector\n"+str(translation_vector))

    #Using the cameras parameters we calculate the rectification parameters for both cameras
    rightRectification, leftRectification, rightProjection, leftProjection, dispartityToDepthMap, rightROI, leftROI = cv.stereoRectify(
                    matrix_right, distortion_coef_right,
                    matrix_left, distortion_coef_left,
                    imageSize, 
                    rotation_matrix, translation_vector,
                    None, None, None, None, None,
                    cv.CALIB_ZERO_DISPARITY)
    
    #Getting the rectificaion matrix for both cameras
    leftMapX, leftMapY = cv.initUndistortRectifyMap(matrix_left, distortion_coef_left, 
                                                    leftRectification, leftProjection,
                                                    imageSize, cv.CV_16SC2)
    rightMapX, rightMapY = cv.initUndistortRectifyMap(matrix_right, distortion_coef_right, 
                                                    rightRectification, rightProjection, 
                                                    imageSize, cv.CV_16SC2)

    #Save the calibration information to further use
    np.savez_compressed("Saved_parameters/retification_informations",
                        imageSize=imageSize,
                        leftMapX=leftMapX,   leftMapY=leftMapY, 
                        rightMapX=rightMapX, rightMapY=rightMapY,
                        leftROI=leftROI,     rightROI=rightROI)
    
    print("Done calibrating and parameters stored at Saved_parameters/retification_informations.npz")


def calibrate_camera(images_names, rows, columns, display_time=10):
    """
    Using chessboard pictures taken with the camera, it determines the camera matrix and distortion coefficients proper to the camera.
    It is recommended to use at least 4 pictures, each picture should show the chessboard from a different angle.
    Chessboard should take most of the images and be completly visible.

    Args:
        images_names (str list): adress list of the pictures used for calibration
        rows (int): number fo rows in chessboard
        columns (int): number of columns in chessboard 
        display_time (int, optional): how long the images will be displayed in ms

    Returns:
        camera_matrix (float matrix): the 3*3 camera matrix
        distortion_coefficients (float list): distortion coefficients of the camera
    """

    assert len(images_names)>0, "List images_names is empty"

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
    
    cv.namedWindow('Chessboard drawing',cv.WINDOW_AUTOSIZE)
    x=0
    for frame in images:
        #makes the frames gray
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        #find the checkerboard, ret states if corners where found
        ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)

        if not(ret):
            print("No corners found in image "+images_names[x])
        
        else:
            # print("corners found in image "+images_names[x])
            #Convolution size used to improve corner detection. Don't make this too large.
            conv_size = (11, 11)
 
            #opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
            
            if display_time>0:
                cv.drawChessboardCorners(frame, (rows,columns), corners, ret)
                cv.imshow('Chessboard drawing', frame)
                cv.waitKey(display_time)
 
            objpoints.append(objp)
            imgpoints.append(corners)
        x+=1
    print("Done finding corners, starting calibration...")
    cv.destroyWindow('Chessboard drawing')
    ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (width, height), None, None)
    print("Done calibrating.")
    print("RMS value single camera: ",ret) 

    return camera_matrix, distortion_coefficients





def stereo_calibrate(mtx1, dist1, mtx2, dist2, frames_names_c1, frames_names_c2, rows, columns, display_time=10):
    """
    Calibrate the two cameras together using the distortion parameter made by the singe calibration function.
    In this step we need to have the pictures from the two camera taken on the same time.
    
    Args:
        mtx2 (MatLike): camera 1 matrix
        dist2 (MatLike): distortion coefficients
        mtx2 (MatLike): camera 2 matrix
        dist2 (MatLike): distortion coefficients
        frames_names_c1 (str list): names of the frames from the camara 1
        frames_names_c2 (str list): names of the frames from the camara 2
        display_time (int, optional): how long the images will be displayed in ms

    Returns:
        R (MatLike): rotation matrix between the two cameras
        T (MatLike): translation vector between the two cameras
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

    #change this to the real world square size (in mm or cm)
    world_scaling = 26 
 
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

        assert ret_c1 and ret_c2, "corners not found in one the two files check if there is a picture where no corners have been found"

        if ret_c1 and ret_c2:
            #determines where the corners are on the frame
            corners_c1 = cv.cornerSubPix(frame_c1, corners_c1, (11, 11), (-1, -1), criteria)
            corners_c2 = cv.cornerSubPix(frame_c2, corners_c2, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints_right.append(corners_c1)
            imgpoints_left.append(corners_c2)

            #Draws chessboard corners on the pictures, only for infomation purpose
            cv.drawChessboardCorners(frame_c1, (rows,columns), corners_c1, ret_c1)
            cv.drawChessboardCorners(frame_c2, (rows,columns), corners_c2, ret_c2)
            scale = 0.4
            frame_c1_resized = cv.resize(frame_c1,None,fx=scale,fy=scale)
            frame_c2_resized = cv.resize(frame_c2,None,fx=scale,fy=scale)
            imOut = np.hstack((frame_c1_resized,frame_c2_resized))
            cv.imshow('Chessboard drawing', imOut)
            cv.waitKey(display_time)
 
    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    
    print("Done finding corners, starting calibration...")
    cv.destroyWindow('Chessboard drawing')
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_right, imgpoints_left, mtx1, dist1,
                                                                 mtx2, dist2, (width, height), criteria = criteria, flags = stereocalibration_flags)
    print("\nRMSE value stereo (<1 perfect, >3 bad)", ret) 
    return R, T



__main__()