import cv2
import time
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

f1 = "Test_pictures/Depth_1/"
f2 = "Test_pictures/Depth_2/"
f3 = "Test_pictures/Depth_3/"

#NOTE: change this
live = 1
folder = f1
#-----------------

if live:
    #Connecting to Camera
    print("Connecting to Camera...")
    cam = cv2.VideoCapture(0)
    # cam.set(cv2.CAP_PROP_FPS, 60)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    frame = cam.read()[1]
    height = frame.shape[0]
    width = frame.shape[1]
    print("Connected to Camera.")
    print("Camera stream width: "+str(width)+"     height: "+str(height))
    assert len(frame)>0, "no frame recieved from camera"

else:
    cam = None
    left = cv2.imread(folder+"L.jpg")
    right = cv2.imread(folder+"R.jpg")
    gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    rectified_gray_left = cv2.remap(gray_left, leftMapX, leftMapY, cv2.INTER_LINEAR )
    rectified_gray_right = cv2.remap(gray_right, rightMapX, rightMapY, cv2.INTER_LINEAR)
    cv2.imshow("image",rectified_gray_left)
    cv2.waitKey(2000)

# assert imageSize == (frame.shape[0],frame.shape[1]),"Frame not same size as the preloaded parameters: "+str(imageSize)+" != "+str((frame.shape[1]//2,frame.shape[0]))

def nothing(x):
    pass

cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',800,800)
cv2.createTrackbar('numDisparities','disp',1,17,nothing) #done
cv2.createTrackbar('blockSize','disp',5,50,nothing)
cv2.createTrackbar('preFilterType','disp',0,1,nothing) #done
cv2.createTrackbar('preFilterSize','disp',0,25,nothing)
cv2.createTrackbar('preFilterCap','disp',1,62,nothing) #done
cv2.createTrackbar('textureThreshold','disp',0,100,nothing)
cv2.createTrackbar('uniquenessRatio','disp',0,100,nothing)
cv2.createTrackbar('speckleRange','disp',0,100,nothing)
cv2.createTrackbar('speckleWindowSize','disp',0,25,nothing)
cv2.createTrackbar('disp12MaxDiff','disp',-1,25,nothing)
cv2.createTrackbar('minDisparity','disp',0,12,nothing) #5,25

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

    numDisparities=stereo.getNumDisparities()
    blockSize=stereo.getBlockSize()
    preFilterType=stereo.getPreFilterType()
    preFilterSize=stereo.getPreFilterSize()
    preFilterCap=stereo.getPreFilterCap()
    textureThreshold=stereo.getTextureThreshold()
    uniquenessRatio=stereo.getUniquenessRatio()
    speckleRange=stereo.getSpeckleRange()
    speckleWindowSize=stereo.getSpeckleWindowSize()
    disp12MaxDiff=stereo.getDisp12MaxDiff()
    minDisparity=stereo.getMinDisparity()

    if live:
        # Reading camera
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

    #Computing depth map
    depth = stereo.compute(rectified_gray_left, rectified_gray_right)
    numDisparities = stereo.getNumDisparities()
    minDisparity = stereo.getMinDisparity()
    depth = (depth/16.0 - minDisparity)/numDisparities

    #Adding color to depth
    # depth = (depth).astype(np.uint8)
    # depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
    
    text = "num:"+str(numDisparities)+", block:"+str(blockSize)+", filter:"+str(preFilterType)+", f_Sz:"+str(preFilterSize)+", f_Cap:"+str(preFilterCap)+", texture:"+str(textureThreshold)+", uRatio"+str(uniquenessRatio)+", sp_Range:"+str(speckleRange)+", sp_Sz:"+str(speckleWindowSize)+", disp:"+str(disp12MaxDiff)+", min:"+str(minDisparity)
    cv2.putText(depth,text,(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)
    # scale = 0.6
    # depth_resize = cv2.resize(depth,None,fx=scale,fy=scale)
    cv2.imshow('img', depth)

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
#	mode=cv2.StereoSGBM_MODE_SGBM
#	#mode=cv2.StereoSGBM_MODE_HH
)
"""
# numDisparities=stereo.getNumDisparities()
    # blockSize=stereo.getBlockSize()
    # preFilterType=stereo.getPreFilterType()
    # preFilterSize=stereo.getPreFilterSize()
    # preFilterCap=stereo.getPreFilterCap()
    # textureThreshold=stereo.getTextureThreshold()
    # uniquenessRatio=stereo.getUniquenessRatio()
    # speckleRange=stereo.getSpeckleRange()
    # speckleWindowSize=stereo.getSpeckleWindowSize()
    # disp12MaxDiff=stereo.getDisp12MaxDiff()
    # minDisparity=stereo.getMinDisparity()