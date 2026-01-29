# 28/11/2025 is working correctly

# This code capture pictures with the stereo camera connected by USB.
# Crops the image in left and right pictures and save them.
# The Objective of this code is to capture the chessboard picture to calibrate the camera.
# place the chessboard close to camera and change the orientation at each picture, make sure the whole chessbaord is visible in both camera.
# Pictures will be taken at 2 secondes interval.
# PS: the camera can take several secondes to start (~50sec)

# for a good calibration take more then 50 pictures

# Wihtout specification frame size, 640*240 ratio 0.375 so for 3840*0.375=1440



import cv2 as cv
import numpy as np


#NOTE: Change these parameters in accordance to your hardware
#----------------------------------------------------
camera_left_ID = 1 # usually 0 is the builtin camera
camera_right_ID = 2
stereo_camera_width = 1280
stereo_camera_height = 720
#----------------------------------------------------3840*1080
resize_scale = 0.6 #0.4 for 1440*3840

folder_left = "Calibration_pictures/Left"
folder_right = "Calibration_pictures/Right"

print("Connecting to Camera...")
camL = cv.VideoCapture(camera_left_ID)
camR = cv.VideoCapture(camera_right_ID)
camL.set(cv.CAP_PROP_FRAME_WIDTH, stereo_camera_width)
camL.set(cv.CAP_PROP_FRAME_HEIGHT, stereo_camera_height)
camR.set(cv.CAP_PROP_FRAME_WIDTH, stereo_camera_width)
camR.set(cv.CAP_PROP_FRAME_HEIGHT, stereo_camera_height)
camL.set(cv.CAP_PROP_AUTOFOCUS, 0)
camR.set(cv.CAP_PROP_AUTOFOCUS, 0)
# camL.set(cv.CAP_PROP_FPS, 60)
# camR.set(cv.CAP_PROP_FPS, 60)
retL,frameL = camL.read()
assert retL, "No frame left readed"
print("Connected to left camera")

retR,frameR = camR.read()
assert retR, "No frame right readed"
print("Connected to right camera")

height = frameL.shape[0]
width = frameR.shape[1]
last_picture_taken = np.hstack((frameL,frameR))
last_picture_taken = cv.resize(last_picture_taken, None, fx=resize_scale, fy=resize_scale)
print("Picture width: "+str(width)+"     height: "+str(height))
print("Connected to cameras")
cv.namedWindow("window",cv.WINDOW_AUTOSIZE)

nb_picture_taken = 0

while (True):
    key = cv.waitKey(1)
    ret = False
    while not(ret):
        retR,right = camR.read()
        retL,left = camL.read()
        ret = retL and retR
    
    # cv.putText(right,"Right",(20,200),cv.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
    # cv.putText(left,"Left",(20,200),cv.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
    frame_live = np.hstack((left,right))
    frame_live = cv.resize(frame_live, None, fx=resize_scale, fy=resize_scale)

    if key == 115: #s
        nb_picture_taken+=1
        last_picture_taken = frame_live
        #crop the frame in left and right pictures
        cv.imwrite(folder_left+"/L_"+str(nb_picture_taken).zfill(3)+".jpg",left)
        cv.imwrite(folder_right+"/R_"+str(nb_picture_taken).zfill(3)+".jpg",right)
        print("picture taken: ",nb_picture_taken)
        cv.waitKey(10)

    elif key == 101: #e
        break
    
    imOut = np.vstack((frame_live,last_picture_taken))
    cv.putText(imOut,"picture number: "+str(nb_picture_taken),(20,50),cv.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
    cv.imshow("window", imOut)

camL.release()
camR.release()
cv.destroyAllWindows()



