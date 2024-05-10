import numpy as np
import cv2 as cv
import glob
import os

#{"numDisparities":[{"numDisparities":1, "blockSize":2, ....}, {"numDisparities":2, "blockSize":2, ....} ] , "blockSize":[....]   }

f1 = "Test_pictures/Depth_1/"
f2 = "Test_pictures/Depth_2/"


#NOTE change this
iterating_values_name = "numDisparities"
folder = f2
# ---------------

#Processing images
left = cv.imread(folder+"L.jpg")
right = cv.imread(folder+"R.jpg")
gray_left = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
gray_right = cv.cvtColor(right, cv.COLOR_BGR2GRAY)

dic = {}

numDisparities_values = [x*16 for x in range(1,17)] #64, S1
blockSize_values = [x*2 + 5 for x in range(5,51)] #21, S2
preFilterType_values = [0,1]
preFilterSize_values = [x*2 + 5 for x in range(0,26)] #9
preFilterCap_values = [x for x in range(1,63)]
textureThreshold_values = [x for x in range(10,101)] #, change rien
uniquenessRatio_values = [x for x in range(0,101)] #15, S3 18 is good

# speckleRange_values = [x for x in range(0,101)] #0, change rien
# speckleWindowSize_values = [x*2 for x in range(0,26)] #0, 
# speckleRange_values = [x*4 for x in range(0,25)] #0, change rien
# speckleWindowSize_values = [x*4 for x in range(0,13)] #0, 
speckleRange_values = [x for x in range(8,12)] #0, change rien
speckleWindowSize_values = [x*4 for x in range(0,13)] #0,
# speckle_range_size_values = [[8,36],[9,40],[11,36],[11,44],[11,48]]

disp12MaxDiff_values = [x for x in range(-1,26)] #-1, S4
minDisparity_values = [x for x in range(0,12)] #0, S5
# print(len(numDisparities_values)*len(blockSize_values)*len(preFilterType_values)*len(preFilterSize_values)*len(preFilterCap_values)*len(textureThreshold_values)*len(uniquenessRatio_values)*len(speckleRange_values)*len(speckleWindowSize_values)*len(disp12MaxDiff_values)*len(minDisparity_values))

# S7 combined
#8, 28
#8, 40

# S8 x, 40 après 20 change plus rien

# S9, x, 36 change rien après 12 (x=[0,16])
#speckle range above 12

# S10 final smaple for speckle range and widowsize 5 parametre
# speckleRange 11  speckleWindowSize36
# speckleRange 9  speckleWindowSize40
# speckleRange 8  speckleWindowSize36
# speckleRange 11  speckleWindowSize44
# speckleRange 11  speckleWindowSize48




if iterating_values_name == "numDisparities":
    iterating_values = numDisparities_values
elif iterating_values_name == "blockSize":
    iterating_values = blockSize_values
elif iterating_values_name == "preFilterType":
    iterating_values =  preFilterType_values
elif iterating_values_name == "preFilterSize":
    iterating_values = preFilterSize_values
elif iterating_values_name == "preFilterCap":
    iterating_values = preFilterCap_values
elif iterating_values_name == "textureThreshold":
    iterating_values = textureThreshold_values
elif iterating_values_name == "uniquenessRatio":
    iterating_values = uniquenessRatio_values
elif iterating_values_name == "speckleRange":
    iterating_values = speckleRange_values
elif iterating_values_name == "speckleWindowSize":
    iterating_values = speckleWindowSize_values
elif iterating_values_name == "disp12MaxDiff":
    iterating_values = disp12MaxDiff_values
elif iterating_values_name == "minDisparity":
    iterating_values = minDisparity_values
else:
    assert 1==0,"not the right key word"


stereo = cv.StereoBM_create()
print("default "+str(stereo.getPreFilterCap()))

depth_list = []

for value  in iterating_values:

    stereo.setNumDisparities(32) #best:80, ok:96,....with prefilter 32,48,64
    stereo.setBlockSize(23) #23,21,19

    """Parameters with prefilter on = 0"""
    stereo.setPreFilterType(0) #0 and 1 is good but different
    stereo.setPreFilterSize(25)#25 is average,13 à 39 si ok
    stereo.setPreFilterCap(13)#12 or 13 is ok but a bit random

    # stereo.setTextureThreshold(50) #changes nearly nothing
    stereo.setUniquenessRatio(16) #16 more or less 5, ... try with 1

    stereo.setSpeckleRange(11) #look couple val but min 8, 12 for sure
    stereo.setSpeckleWindowSize(44) #look couple val
    
    stereo.setDisp12MaxDiff(1) #-1,0,1
    stereo.setMinDisparity(4) #3is best,0 à 5 ...even with prefilter but higher can be tolerable

    

    


    
    # stereo.setUniquenessRatio(18)
    # stereo.setDisp12MaxDiff(1)
    
    #S6 with speckcle range 10


    if iterating_values_name == "numDisparities":
        stereo.setNumDisparities(value)
    elif iterating_values_name == "blockSize":
        stereo.setBlockSize(value)
    elif iterating_values_name == "preFilterType":
        print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeuhhhhhhhhhhhhhhhh")
        stereo.setPreFilterType(value)
    elif iterating_values_name == "preFilterSize":
        stereo.setPreFilterSize(value)
    elif iterating_values_name == "preFilterCap":
        stereo.setPreFilterCap(value)
    elif iterating_values_name == "textureThreshold":
        stereo.setTextureThreshold(value)
    elif iterating_values_name == "uniquenessRatio":
        stereo.setUniquenessRatio(value)
    elif iterating_values_name == "speckleRange":
        stereo.setSpeckleRange(value)
    elif iterating_values_name == "speckleWindowSize":
        stereo.setSpeckleWindowSize(value)
    elif iterating_values_name == "disp12MaxDiff":
        stereo.setDisp12MaxDiff(value)
    elif iterating_values_name == "minDisparity":
        stereo.setMinDisparity(value)
    else:
        assert 1==0,"not the right key word"

    


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
