import numpy as np
import cv2
import glob

#NOTE change folder 
folder = 3
#------------------

file_path = glob.glob("Test_pictures/Depth_2/Depth/Sample_"+str(folder)+"/*")[0]
print(file_path)

load = np.load(file_path,allow_pickle=True)
# depth_list = load["frames_list"]
# depth_list_name = load["frames_names"]
depth_dic = load["depth_dic"].item()

parameter = list(depth_dic.keys())[0]
print(parameter)
depth_list = list(depth_dic.values())[0]

depth_list_name = [parameter+" "+str(depth[parameter]) for depth in depth_list]
print(depth_list_name)

cv2.namedWindow('disp',cv2.WINDOW_AUTOSIZE)
key  = 0
im_id = 0

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
    
    if key != -1:
        imOut = depth_list[im_id]["frame"]
        text = depth_list_name[im_id]
        # cv2.putText(imOut,text,(20,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2)
        cv2.imshow("disp",imOut)
  


cv2.destroyAllWindows()


