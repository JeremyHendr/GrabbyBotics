import cv2
import glob
import numpy as np



#Load rectification informations
calibration = np.load("Saved_parameters/retification_informations.npz", allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightROI = tuple(calibration["rightROI"])
rightMapY = calibration["rightMapY"]

#Connecting to Camera
print("Connecting to Camera...")
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FPS, 60)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
frame = cam.read()[1]
height = frame.shape[0]
width = frame.shape[1]
print("Connected to Camera.")
print("Picture width: "+str(width)+"     height: "+str(height))

assert len(frame)>0, "no frame recieved from camera"

cv2.namedWindow('disp',cv2.WINDOW_AUTOSIZE)

stereo = cv2.StereoBM_create()

while True:
    #Reading camera
    ret = 0
    while not(ret): 
        ret,frame = cam.read(1)
    
    #Cropping frame and make it gray
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_left = gray_frame[0:height,0:int(width/2)]
    gray_right = gray_frame[0:height,int(width/2):(width)]

    #Rectifying frames using preloaded parameters
    rectified_gray_left = cv2.remap(gray_left, leftMapX, leftMapY, cv2.INTER_LINEAR )
    rectified_gray_right = cv2.remap(gray_right, rightMapX, rightMapY, cv2.INTER_LINEAR)

    #NOTE: uncomment following code to show difference between rectified and normal frames in a separated window
    """
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
    """

    stereo.setNumDisparities(32) #best:80, ok:96,....with prefilter 32,48,64
    stereo.setBlockSize(19) #23,21,19
    stereo.setPreFilterType(0) #0 and 1 is good but different
    stereo.setPreFilterSize(25)#25 is average,13 à 39 si ok
    stereo.setPreFilterCap(13)#12 or 13 is ok but a bit random
    stereo.setTextureThreshold(10) #changes nearly nothing
    stereo.setUniquenessRatio(18) #16 more or less 5, ... try with 1
    stereo.setSpeckleRange(11) #look couple val but min 8, 12 for sure
    stereo.setSpeckleWindowSize(44) #look couple val
    stereo.setDisp12MaxDiff(1) #-1,0,1
    stereo.setMinDisparity(4) #3is best,0 à 5 ...even with prefilter but higher can be tolerable

    depth = stereo.compute(gray_left, gray_right)
    numDisparities = stereo.getNumDisparities()
    minDisparity = stereo.getMinDisparity()
    depth = (depth/16.0 - minDisparity)/numDisparities

    cv2.imshow('disp', depth)

    if cv2.waitKey(1) == 101: #"e" pressed
            break

cam.release()
cv2.destroyAllWindows()