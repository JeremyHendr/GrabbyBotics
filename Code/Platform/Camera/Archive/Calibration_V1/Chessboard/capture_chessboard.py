import numpy as np
import cv2
import time

print("Starting....")
camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FPS, 120)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("Camera runnning and Setup!")

print("Startig capture in 3sec ...")
time.sleep(3)

for i in range(10):
    _,frame = camera.read()
    cv2.imwrite("visualisation_calibration/Calibration_pictures/"+str(i)+".jpg",frame)
    print("picture "+str(i))
    time.sleep(2)

    if cv2.waitKey(1) & 0xFF == ord('w'):
        break


camera.release()
cv2.destroyAllWindows()