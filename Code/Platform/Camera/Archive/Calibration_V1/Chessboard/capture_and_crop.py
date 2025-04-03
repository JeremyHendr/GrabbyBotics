import cv2 as cv


#LEFT CAMERA IS THE ONE WHEN WE LOOK FROM THE POINT OF VIEW OF THE CAMERA
#----------------------------------------------------
folder_left = "Calibration_V2/pictures/Left/"
folder_right = "Calibration_V2/pictures/Right/"
picture_nb = 5
#----------------------------------------------------

cam = cv.VideoCapture(1)
cam.set(cv.CAP_PROP_FPS, 120)

cam.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
frame = cam.read()[1]
height = frame.shape[0]
width = frame.shape[1]
print("Picture width: "+str(width)+"     height: "+str(height))

x=0
while (True):
    frame = cam.read()[1]
    left=frame[0:height,0:int(width/2)]
    right=frame[0:height,int(width/2):(width)]
    cv.imshow('left',left)
    cv.imshow('Right',right)

    if x<=picture_nb :
        print(x)
        cv.imwrite(folder_left+str(x)+"_L.jpg",left)
        cv.imwrite(folder_right+str(x)+"_R.jpg",right)
        x+=1

    if cv.waitKey(1) & 0xFF == ord('w'):
        break


cam.release()
cv.destroyAllWindows()