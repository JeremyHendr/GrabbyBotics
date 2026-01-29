import numpy as np
import cv2 as cv
import glob
import os

#{"numDisparities":[{"numDisparities":1, "blockSize":2, ....}, {"numDisparities":2, "blockSize":2, ....} ] , "blockSize":[....]   }

f1 = "Test_pictures/Depth_1/"
f2 = "Test_pictures/Depth_2/"


#NOTE change this
iterating_values_name = "numDisparities"
folder = f1
# ---------------

#Processing images
left = cv.imread(folder+"L.jpg")
right = cv.imread(folder+"R.jpg")
gray_left = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
gray_right = cv.cvtColor(right, cv.COLOR_BGR2GRAY)

file_path = glob.glob("Test_pictures/Depth_2/Depth/Last saved parameters/*")[0]
print(file_path)

load = np.load(file_path,allow_pickle=True)
# depth_list = load["frames_list"]
# depth_list_name = load["frames_names"]
dic = load["depth_dic"].item()
depth_list = dic[iterating_values_name]

stereo = cv.StereoBM_create()


al = [32,64] #num
al = [depth["numDisparities"] for depth in depth_list]

bl = [19,23] #block
bl = [depth["blockSize"] for depth in depth_list]

cl = [1,14,16,18] #ratio
cl = [depth["uniquenessRatio"] for depth in depth_list]

dl = [11,44] #range
dl = [depth["speckleRange"] for depth in depth_list]

el = [44] #spSz
el = [depth["speckleWindowSize"] for depth in depth_list]

fl = [-1,1] #disp
fl = [depth["disp12MaxDiff"] for depth in depth_list]

gl = [4,10]
gl = [depth["minDisparity"] for depth in depth_list]


"""Parameters with prefilter on = 0"""
stereo.setPreFilterType(0) #0 and 1 is good but different
stereo.setPreFilterSize(25)#25 is average,13 à 39 si ok
stereo.setPreFilterCap(13)#12 or 13 is ok but a bit random

print(depth_list[3])
for depth in depth_list:
    cv.imshow("img",depth["frame"])
    cv.waitKey(3000)


"""
for a  in al:
    stereo.setNumDisparities(a) #best:80, ok:96,....with prefilter 32,48,64
    for b in bl:
        stereo.setBlockSize(b) #23,21,19
        for c in cl:
            stereo.setUniquenessRatio(c) #16 more or less 5, ... try with 1
            for d in dl:
                stereo.setSpeckleRange(d) #look couple val but min 8, 12 for sure
                for e in el:
                    stereo.setSpeckleWindowSize(e) #look couple val
                    for f in fl:
                        stereo.setDisp12MaxDiff(f) #-1,0,1
                        for g in gl:
                            stereo.setMinDisparity(g) #3is best,0 à 5 ...even with prefilter but higher can be tolerable

                            depth = stereo.compute(gray_left, gray_right)
                            numDisparities = stereo.getNumDisparities()
                            minDisparity = stereo.getMinDisparity()
                            depth = (depth/16.0 - minDisparity)/numDisparities

                            depth_list.append({ "numDisparities":stereo.getNumDisparities(),
                                                "blockSize":stereo.getBlockSize(),
                                                "preFilterType":stereo.getPreFilterType(),
                                                "preFilterSize":stereo.getPreFilterSize(),
                                                "preFilterCap":stereo.getPreFilterCap(),
                                                "textureThreshold":stereo.getTextureThreshold(),
                                                "uniquenessRatio":stereo.getUniquenessRatio(),
                                                "speckleRange":stereo.getSpeckleRange(),
                                                "speckleWindowSize":stereo.getSpeckleWindowSize(),
                                                "disp12MaxDiff":stereo.getDisp12MaxDiff(),
                                                "minDisparity":stereo.getMinDisparity(),
                                                "frame":depth})
                            # print("one more "+str(stereo.getMinDisparity()))
print(len(depth_list))
"""
dic[iterating_values_name] = []
depth_to_be_saved = []
depth_already_saved = []
# depth_list_name = [iterating_values_name+" "+str(depth[iterating_values_name]) for depth in depth_list]
depth_list_name = ["speckleRange"+" "+str(depth["speckleRange"])+"  speckleWindowSize"+str(depth["speckleWindowSize"]) for depth in depth_list]
cv.namedWindow('disp',cv.WINDOW_AUTOSIZE)
key  = 0
im_id = 0

while key != 101:
    key = cv.waitKey(50)
    if key == 100: #d
        im_id += 1
        if im_id >=len(depth_list):
            im_id = len(depth_list)-1

    elif key == 113:#q
        im_id -= 1
        if im_id < 0:
            im_id = 0
    
    elif key == 115: #s
        if not(im_id in depth_already_saved):
            depth_to_be_saved.append(depth_list[im_id])
            depth_already_saved.append(im_id)
        
    if key != -1:
        imOut = depth_list[im_id]["frame"]
        text = "num:"+str(depth_list[im_id]["numDisparities"])+", block:"+str(depth_list[im_id]["blockSize"])+", filter:"+str(depth_list[im_id]["preFilterType"])+", f_Sz:"+str(depth_list[im_id]["preFilterSize"])+", f_Cap:"+str(depth_list[im_id]["preFilterCap"])+", texture:"+str(depth_list[im_id]["textureThreshold"])+", uRatio"+str(depth_list[im_id]["uniquenessRatio"])+", sp_Range:"+str(depth_list[im_id]["speckleRange"])+", sp_Sz:"+str(depth_list[im_id]["speckleWindowSize"])+", disp:"+str(depth_list[im_id]["disp12MaxDiff"])+", min:"+str(depth_list[im_id]["minDisparity"])
        cv.putText(imOut,text,(20,50),cv.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        cv.imshow("disp",imOut)
  
cv.destroyAllWindows()
if len(depth_to_be_saved)>0:
    print(str(len(depth_to_be_saved))+" parameters saved")
    for i in depth_already_saved:
        print(depth_list_name[i])

    dic[iterating_values_name] = depth_to_be_saved
    existing_folders = glob.glob(folder+"Depth/*")
    new_folder = folder+"Depth/Sample_"+str(len(existing_folders)+1)
    os.mkdir(new_folder)
    np.savez_compressed(new_folder+"/"+iterating_values_name+".npz",depth_dic=dic)
    print("Frames saved at "+new_folder)
