import cv2
import glob
import numpy as np
import os


cv2.namedWindow('disp',cv2.WINDOW_AUTOSIZE)

#Load rectification informations
calibration = np.load("Saved_parameters/retification_informations.npz", allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightROI = tuple(calibration["rightROI"])
rightMapY = calibration["rightMapY"]


#NOTE: choose folder
f1 = "Test_pictures/Depth_1/"
f2 = "Test_pictures/Depth_2/"
folder = f1

#Processing images
left = cv2.imread(folder+"L.jpg")
right = cv2.imread(folder+"R.jpg")
gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
# rectified_gray_left = cv2.remap(gray_left, leftMapX, leftMapY, cv2.INTER_LINEAR )
# rectified_gray_right = cv2.remap(gray_right, rightMapX, rightMapY, cv2.INTER_LINEAR)

stereo = cv2.StereoBM_create()

#NOTE: all parameters available
# numDisparities: 1 to 17 (*16)
# blockSize: 5 to 50 (*2 + 5)
# preFilterType: 0 or 1
# preFilterSize: 2 to 25 (*2 + 5)
# preFilterCap: 5 to 62
# textureThreshold: 10 to 100
# uniquenessRatio: 15 to 100
# speckleRange: 0 to 100
# speckleWindowSize: 3 to 25 (*2)
# disp12MaxDiff: 5 to 25
# minDisparity: 5 to 12

numDisparities = 0
blockSize = 0
preFilterType = 0
preFilterSize = 0
preFilterCap = 0
textureThreshold = 0
uniquenessRatio = 0
speckleRange = 0
speckleWindowSize = 0
disp12MaxDiff = 0
minDisparity = 0

# stereo.setNumDisparities(numDisparities)
# stereo.setBlockSize(blockSize)
# stereo.setPreFilterType(preFilterType)
# stereo.setPreFilterSize(preFilterSize)
# stereo.setPreFilterCap(preFilterCap)
# stereo.setTextureThreshold(textureThreshold)
# stereo.setUniquenessRatio(uniquenessRatio)
# stereo.setSpeckleRange(speckleRange)
# stereo.setSpeckleWindowSize(speckleWindowSize)
# stereo.setDisp12MaxDiff(disp12MaxDiff)
# stereo.setMinDisparity(minDisparity)


depth_list = []
depth_list_name = []


for x in range(1,17):
    i = x*16
    stereo.setNumDisparities(i)

    #Caluclating depth
    depth = stereo.compute(gray_left, gray_right)

    """depth first formula"""
    numDisparities = stereo.getNumDisparities()
    minDisparity = stereo.getMinDisparity()
    depth = (depth/16.0 - minDisparity)/numDisparities
    depth_list.append(depth)
    depth_list_name.append("numDisparities "+str(i))







saved_depth = {}

key  = 0
im_id = 0
is_frame_list_saved = False
while key != 101:
    key = cv2.waitKey(50)
    if key == 100: #d
        im_id += 1
        if im_id >=len(depth_list):
            im_id = len(depth_list)-1

    elif key == 113:#q
        im_id -= 1
        if im_id < 0:
            im_id = 0
    
    elif key == c: #s
        #NOTE: saves only the concerned image in a list (saves them all at the end)

        #NOTE: saves all images
        #save images
        existing_folders = glob.glob(folder+"Depth/*")
        new_folder = folder+"Depth/Sample_"+str(len(existing_folders)+1)
        os.mkdir(new_folder)
        np.savez_compressed(new_folder+"/depth_results.npz",frames_list=depth_list,frames_names=depth_list_name)
        print("Frames saved at "+new_folder)
        is_frame_list_saved = True


    if key != -1:
        imOut = depth_list[im_id]
        text = depth_list_name[im_id]
        cv2.putText(imOut,text,(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        cv2.imshow("disp",imOut)
  


cv2.destroyAllWindows()

"""last method should be tried alone"""
#"q"=113
#"d"=100

# test = cv2.imread("Test_pictures/Depth_1/Depth/Sample_1/scalar 256.jpg")
# print(test)
# cv2.imshow("test",test)
# cv2.waitKey(3000)

"""depth by scalar"""
#NOTE: 256 or 512 are the best
# alpha = 256
# depth_1 = depth/alpha
# depth_list.append(depth_1)
# depth_list_name.append("scalar 256")
# alpha = 512
# depth_1 = depth/alpha
# depth_list.append(depth_1)
# depth_list_name.append("scalar 512")

# for x in range(1,14):
#     depth_list.append(depth/(2**x))
#     depth_list_name.append("scalar "+str(2**x))


# depth_2 = (depth/16) / max_disparity
# depth = (depth).astype(np.uint8)
# depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)

 #TODO: put depth map and original fram side by side
#scale = 0.6
# depth_resize = cv2.resize(depth,None,fx=scale,fy=scale)

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




