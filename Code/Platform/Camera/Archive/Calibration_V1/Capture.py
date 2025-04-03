import cv2
import time

print("Starting....")
camera = cv2.VideoCapture(1)
#camera is 3840*1080
CAMERA_WIDTH = 1080
CAMERA_HEIGHT = 720
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
print("Camera runnning and Setup!")


# Different directories for each camera
directory = "Calibration_pictures/{:06d}.jpg"

print("starting capturing in 3sec")
time.sleep(3)
for x in range(1, 10):
    # Actually save the frames
    _,frame = camera.read()
    cv2.imwrite("Calibration_pictures/"+str(x)+".jpg", frame)
    print(frame)
    time.sleep(0.01)
    print(x)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Done capturing images")
camera.release()
cv2.destroyAllWindows()