import cv2

print("Starting....")
camera = cv2.VideoCapture(1)
#camera is 3840*1080
CAMERA_WIDTH = 1080
CAMERA_HEIGHT = 720
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
print("Camera runnning and Setup!")

while(True):
    # print("loop")
    # camera.grab() takes the pictures from the camera
    # camera.retrieve() decode the picture
    # camera.read() does both but with two camera it is bether to do it step by step to reduce lag

    # if not (camera.grab()):
    #     print("No more frames")
    #     break
    # _,leftFrame = camera.retrieve()
    
    _,frame = camera.read()
    cv2.imshow('Setreo camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    

camera.release()
cv2.destroyAllWindows()