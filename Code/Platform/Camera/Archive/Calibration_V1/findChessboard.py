import cv2
import numpy as np
import time
"""
cb_shape = (7, 6)

img = cv2.imread("visualisation_calibration/Calibration_pictures/0.jpg")
print(img)
cv2.imshow("original",img)

# corners = cv2.findChessboardCorners(img, cb_shape, cv2.CALIB_CB_FAST_CHECK)[1]
# cv2.imshow("corners",corners)

# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
# cv2.drawChessboardCorners(img, cb_shape, corners, 1)

while True:
    time.sleep(1)
"""

print("Starting....")
camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FPS, 120)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
frame = camera.read()[1]
height, width, _ = frame.shape
print("Camera runnning and Setup!")

x=0
while (True):
    frame = camera.read()[1]
    left=frame[0:height,0:int(width/2)]
    right=frame[0:height,int(width/2):(width)]
    cv2.imshow('left',left)
    cv2.imshow('Right',right)
    # if x<=10 :
    #     print(x)
    #     cv2.imwrite("visualisation_calibration/Calibration_pictures/"+str(x)+"_L.jpg",left)
    #     cv2.imwrite(str(x)+"_R.jpg",right)
    #     x+=1

    if cv2.waitKey(1) & 0xFF == ord('w'):
        break


camera.release()
cv2.destroyAllWindows()
