"""
This script shows in the same window the left and right output from the camera at the top and the rectified version at the bottom.
To test is the rectification is correct place the camera close to straight lines and check if they are really straight.

Change the camera ID if needed.

Code may take up to 1 minute to launch. (connexion to camera takes a while).

Press "e" to quit the window and stop the execution.
"""

import cv2
import time
import numpy as np


#NOTE: change this in a manner that it suits you
Camera_ID = 0

#Connecting to Camera
print("Connecting to Camera...")
cam = cv2.VideoCapture(Camera_ID)
# cam.set(cv2.CAP_PROP_FPS, 60)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
frame = cam.read()[1]
height = frame.shape[0]
width = frame.shape[1]
print("Connected to Camera.")
print("Picture width: "+str(width)+"     height: "+str(height))

assert len(frame)>0, "no frame recieved from camera"

#Load rectification informations
calibration = np.load("Saved_parameters/retification_informations.npz", allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightROI = tuple(calibration["rightROI"])
rightMapY = calibration["rightMapY"]

assert imageSize == (width/2,height),"Frame is not the same size as the preloaded parameters"

while True:
    #Reading camera
    ret = 0
    while not(ret): 
        ret,frame = cam.read()
    
    #Cropping frame and make it gray
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_left = gray_frame[0:height,0:int(width/2)]
    cv2.putText(gray_left,"left",(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)
    gray_right = gray_frame[0:height,int(width/2):(width)]
    cv2.putText(gray_right,"Right",(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)

    #Rectifying frames using preloaded parameters
    rectified_gray_left = cv2.remap(gray_left, leftMapX, leftMapY, cv2.INTER_LINEAR )
    rectified_gray_right = cv2.remap(gray_right, rightMapX, rightMapY, cv2.INTER_LINEAR)
    cv2.putText(rectified_gray_left,"rect_left",(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)
    cv2.putText(rectified_gray_right,"rect_right",(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)

    #show difference between rectified and normal frames
    scale = 0.4
    #rectified resized and stacked
    left_rectified_resized = cv2.resize(rectified_gray_left, None, fx=scale, fy=scale)
    right_rectified_resized = cv2.resize(rectified_gray_right, None, fx=scale, fy=scale)
    rectified_stack = np.hstack((left_rectified_resized,right_rectified_resized))
  
    #normal resized and stacked
    left_resized = cv2.resize(gray_left, None, fx=scale, fy=scale)
    right_resized = cv2.resize(gray_right, None, fx=scale, fy=scale)
    normal_stack = np.hstack((left_resized,right_resized))
    
    #stack normal and rectified
    stacked_image = np.vstack((normal_stack,rectified_stack))
    cv2.imshow("Camera",stacked_image)
    
    if cv2.waitKey(1) == 101: #"e" pressed
            break

cam.release()
cv2.destroyAllWindows()
