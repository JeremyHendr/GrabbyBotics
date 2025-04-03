"""
This code capture pictures with the stereo camera connected by USB
Crops the image in the left and right pictures and place them accordingly in their folder
The Objective of this code is to capture the chessboard picture to calibrate the cameras.
place the chessboard close to camera and change the orientation at each picture, make sure the chessbaord is visible in both camera
5 pictures at 2 secondes interval will be taken.
"""

import cv2 as cv
import time

#LEFT CAMERA IS THE ONE WHEN WE LOOK FROM THE POINT OF VIEW OF THE CAMERA
#----------------------------------------------------
folder = "Test_pictures/Depth_3/Test"
picture_nb = 2
#----------------------------------------------------

print("Connecting to Camera...")
cam = cv.VideoCapture(0)
cam.set(cv.CAP_PROP_FPS, 120)
cam.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
frame = cam.read()[1]
height = frame.shape[0]
width = frame.shape[1]
print("Picture width: "+str(width)+"     height: "+str(height))
print("Connected to camera, starting in 3sec...")
time.sleep(3)

x=0
while (True):
    frame = cam.read()[1]
    left=frame[0:height,0:int(width/2)]
    right=frame[0:height,int(width/2):(width)]
    cv.imshow('left',left)
    cv.imshow('Right',right)

    if x<picture_nb :
        print("Picture "+str(x))
        cv.imwrite(folder+"/Left/"+str(x)+"_L.jpg",left)
        cv.imwrite(folder+"/Right/"+str(x)+"_R.jpg",right)
        x+=1
        time.sleep(2)
    else:
        break

    if cv.waitKey(1) & 0xFF == ord('w'):
        break


cam.release()
cv.destroyAllWindows()