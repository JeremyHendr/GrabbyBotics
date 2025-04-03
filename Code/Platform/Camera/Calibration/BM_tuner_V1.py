"""
This script is made to tune the stereo BM (Block Matcher), you need to find the parameters that suits you the most.
When you found parameters that suits press "s" to save a frame and the according parameters in an external folder.
After testing multiple arrangements you can then choose the best one.
When running, you will see in on the left the depth map and on the right the left frame.
"""

import cv2
import time
import numpy as np

#Connecting to Camera
print("Connecting to Camera...")
cam = cv2.VideoCapture(1)
cam.set(cv2.CAP_PROP_FPS, 60)
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



#Creating tackbars to tune Stereo BM
def nothing(x):
    pass

cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)
cv2.createTrackbar('numDisparities','disp',1,17,nothing) #done
cv2.createTrackbar('blockSize','disp',5,50,nothing)
cv2.createTrackbar('preFilterType','disp',0,1,nothing) #done
cv2.createTrackbar('preFilterSize','disp',2,25,nothing)
cv2.createTrackbar('preFilterCap','disp',5,62,nothing) #done
cv2.createTrackbar('textureThreshold','disp',10,100,nothing)
cv2.createTrackbar('uniquenessRatio','disp',15,100,nothing)
cv2.createTrackbar('speckleRange','disp',0,100,nothing)
cv2.createTrackbar('speckleWindowSize','disp',3,25,nothing)
cv2.createTrackbar('disp12MaxDiff','disp',5,25,nothing)
cv2.createTrackbar('minDisparity','disp',5,12,nothing) #5,25


stereo = cv2.StereoBM_create()

while True:
    # Updating the parameters based on the trackbar positions
    numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16
    blockSize = cv2.getTrackbarPos('blockSize','disp')*2 + 5
    preFilterType = cv2.getTrackbarPos('preFilterType','disp')
    preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')*2 + 5
    preFilterCap = cv2.getTrackbarPos('preFilterCap','disp')
    textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
    uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
    speckleRange = cv2.getTrackbarPos('speckleRange','disp')
    speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')*2
    disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')
    minDisparity = cv2.getTrackbarPos('minDisparity','disp')

    stereo.setNumDisparities(numDisparities)
    stereo.setBlockSize(blockSize)
    stereo.setPreFilterType(preFilterType)
    stereo.setPreFilterSize(preFilterSize)
    stereo.setPreFilterCap(preFilterCap)
    stereo.setTextureThreshold(textureThreshold)
    stereo.setUniquenessRatio(uniquenessRatio)
    stereo.setSpeckleRange(speckleRange)
    stereo.setSpeckleWindowSize(speckleWindowSize)
    stereo.setDisp12MaxDiff(disp12MaxDiff)
    stereo.setMinDisparity(minDisparity)

    #Reading camera
    ret = 0
    while not(ret): 
        ret,frame = cam.read()
    
    #Cropping frame and make it gray
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_left = gray_frame[0:height,0:int(width/2)]
    gray_right = gray_frame[0:height,int(width/2):(width)]

    #Rectifying frames using preloaded parameters
    rectified_gray_left = cv2.remap(gray_left, leftMapX, leftMapY, cv2.INTER_LINEAR )
    rectified_gray_right = cv2.remap(gray_right, rightMapX, rightMapY, cv2.INTER_LINEAR)

    #compute stereo frame
    depth = stereo.compute(rectified_gray_left, rectified_gray_right)
    depth = (depth).astype(np.uint8)
    depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
    
    #TODO: put depth map and original fram side by side
    #scale = 0.6
    # depth_resize = cv2.resize(depth,None,fx=scale,fy=scale)

    #TODO: find the right formula to apply to depth
    #divided by a "random" scalar, power of 2
    depth_1 = depth/16

    #
    depth_2 = (depth/16) / max_disparity
    cv2.imshow('disp', depth)

    if cv2.waitKey(1) == 101: #"e" pressed
            break

cam.release()
cv2.destroyAllWindows()



#NOTE:
"""
blockSize = 11
min_disparity = 50
max_disparity = 150
P1 = 8
P2 = 32

stereo = cv2.StereoSGBM_create(
	minDisparity=min_disparity,
	numDisparities=(max_disparity - min_disparity),
	blockSize=blockSize,
	P1=3*blockSize*blockSize * P1,
	P2=3*blockSize*blockSize * P2,
	disp12MaxDiff=0,
	preFilterCap=0,
	uniquenessRatio=10,
#	speckleWindowSize=100,
#	speckleRange=1,
#	mode=cv.StereoSGBM_MODE_SGBM
#	#mode=cv.StereoSGBM_MODE_HH
)
"""

"""
out = (disparity / np.float32(16)) / max_disparity
"""