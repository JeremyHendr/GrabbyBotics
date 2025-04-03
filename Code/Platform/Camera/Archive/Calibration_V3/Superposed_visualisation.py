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
image_name_left = "a"
image_name_right =  "a"
left = cv.imread(image_name_left)
right = cv.imread(image_name_right)
height = left.shape[0]
width = left.shape[1]

superposed_image = cv.addWeighted(left, 0.5, right, 0.5, 0, None)
cv.imshow("Camera",superposed_image)
cv.waitKey(5000)

# cam.release()
cv.destroyAllWindows()
