import cv2
import time
import numpy as np

fixedLeft = cv2.imread("Calibration_V2/pictures/depth_mat_test/tune_BM/left.jpg")
fixedRight = cv2.imread("Calibration_V2/pictures/depth_mat_test/tune_BM/right.jpg")
grayLeft = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
grayRight = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)

def nothing(x):
    pass



#NOTE: numDisparities
#range: 1 to 17 (multiple of 16)
#1 is perfect, more becomes somber

#NOTE: preFilterType
#range 0 or 1
#0 is perfect, 1 loses a lot of detail 

#NOTE: preFilterCap
#range: 5 to 62 (*2 + 5)
#10 seams good, but more would be good to remove some small isolated errors

#NOTE uniquenessRatio
#range 15 to 100
#between 5 and 25 is alright
#12 seems the best


cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)
#1 is perfect not more
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

    depth = stereo.compute(grayLeft, grayRight)
    depth = (depth).astype(np.uint8)
    depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
    
    # # Converting to float32 
    # depth = depth.astype(np.float32)

    # # Scaling down the disparity values and normalizing them 
    # depth = (depth/16.0 - minDisparity)/numDisparities
  

    cv2.imshow('disp', depth)
    # print("numDisparities: "+str(numDisparities))
    
    time.sleep(0.01)
    
    if cv2.waitKey(1) == 27:
            break






