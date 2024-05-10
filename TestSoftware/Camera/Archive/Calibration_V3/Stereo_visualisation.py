"""
, press: q for exit, m for mono and s for separated
"""

import cv2 as cv
import numpy as np

# print("Connecting to Camera...")
# cam = cv.VideoCapture(1)
# cam.set(cv.CAP_PROP_FPS, 60)
# cam.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
# cam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

# frame = cam.read()[1]
# height = frame.shape[0]
# width = frame.shape[1]
# print("Combined lens shape, width: "+str(width)+"     height: "+str(height))
# print("Connected to camera.")


cv.namedWindow('Camera',cv.WINDOW_NORMAL)

#test images
left = cv.imread("Test_pictures/0_L.jpg")
right = cv.imread("Test_pictures/0_R.jpg")
# scale = 0.5
# print(left.shape)
# image_size = (int(left.shape[1]*scale),int(scale*left.shape[0]))
# print(image_size)
# left = cv.resize(left,image_size)
# right = cv.resize(right,image_size)
# print(left.shape)
display = "separated"

# imOut = np.uint32(left / 2 + right / 2)
# cv.imshow("Camera",imOut)
height = left.shape[0]
width = left.shape[1]
a = 40
left = left[0:height,a:width]
right = right[0:height,0:width-a]
superposed = cv.addWeighted(left, 0.7, right, 0.3, 0, None)
cv.imshow("Camera",superposed)
cv.waitKey(3000)



while (True):
    pass
    # frame = cam.read()[1]
    # left = frame[0:height,0:int(width/2)]
    # right = frame[0:height,int(width/2):(width)]
    

    # if display == "mono":
    #     imOut = np.uint32(left / 2 + right / 2)
    #     cv.imshow("Camera",imOut)
    # else:
    #     cv.imshow("Camera",left)
    #     cv.imshow("Camera",right)

    # im = np.hstack((left,right))
    # cv.imshow("Camera",im)

    # cv.imshow("Camera",right)

    

    key = cv.waitKey(1)
    if key == 101: #exit
        break
    
    elif key == "m": #mono, both frame supperposed
        display = "mono"

    elif key == "s": #separated, both frame side by side
        display = "separated"



# cam.release()
cv.destroyAllWindows()
