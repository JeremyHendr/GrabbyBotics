import os
import sys
import numpy as np
import cv2 as cv; cv2 = cv

f1 = "Test_pictures/Depth_1/"
f2 = "Test_pictures/Depth_2/"
f3 = "Test_pictures/Depth_3/"

#NOTE: CHANGE THIS PARAMETERS
live = 0
folder = f3
#----------------------------

if not(live):    
    imgL = cv.imread(folder+"L.jpg",0)
    imgR = cv.imread(folder+"R.jpg",0)
else:
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


blockSize = 11
min_disparity = 50
max_disparity = 150
P1 = 8
P2 = 32

stereo = cv.StereoSGBM_create(
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

while True:
	if live:
        ret,frame = cam.read()
		while not(ret):
			ret,frame = cam.read()

    if cv.waitKey() == 101:
		break


def redraw():
	disparity = stereo.compute(imgL,imgR)

	out = (disparity / np.float32(16)) / max_disparity
	
	scale = 0.6
	out = cv.resize(out,None,fx=scale,fy=scale)
	
	cv.imshow("out", out)


def on_blockSize(pos):
	global blockSize
	blockSize = 1 - (1 - pos) // 2 * 2
	stereo.setBlockSize(blockSize)
	redraw()

def on_mindisparity(pos):
	global min_disparity
	min_disparity = pos
	stereo.setMinDisparity(min_disparity)
	stereo.setNumDisparities(max_disparity - min_disparity)
	redraw()

def on_maxdisparity(pos):
	global max_disparity
	max_disparity = pos
	stereo.setNumDisparities(max_disparity - min_disparity)
	redraw()

def on_P1(pos):
	global P1
	P1 = pos
	stereo.setP1(3*blockSize*blockSize * P1)
	redraw()

def on_P2(pos):
	global P2
	P2 = pos
	stereo.setP2(3*blockSize*blockSize * P2)
	redraw()

def set_disp12MaxDiff(pos):
	stereo.setDisp12MaxDiff(pos)
	redraw()

def set_preFilterCap(pos):
	stereo.setPreFilterCap(pos)
	redraw()


cv.namedWindow("out",cv.WINDOW_AUTOSIZE)
cv.createTrackbar("blockSize", "out", blockSize, 51, on_blockSize)
cv.createTrackbar("minDisparity", "out", min_disparity, 200, on_mindisparity)
cv.createTrackbar("maxDisparity", "out", max_disparity, 200, on_maxdisparity)
cv.createTrackbar("P1", "out", P1, 64, on_P1)
cv.createTrackbar("P2", "out", P2, 64, on_P2)
cv.createTrackbar("disp12MaxDiff", "out", 0, 100, set_disp12MaxDiff)
cv.createTrackbar("preFilterCap", "out", 0, 100, set_preFilterCap)



redraw()
while True:
	key = cv.waitKey()
	if key == -1:
		break
	print(key)