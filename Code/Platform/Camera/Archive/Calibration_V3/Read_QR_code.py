import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode

#TODO: video capture

frame = cv.imread("Test_pictures/QR_code/1.jpg")
print("Image size: "+str(frame.shape))

# upper_left_part = frame[0:100,0:100]
# cv.imshow("upper left part",upper_left_part)
# cv.waitKey(5000)

while True:
    # ret,frame = cam.read()

    for barcode in decode(frame):
        print("barcode: "+str(barcode))
        print("Raw data: "+str(barcode.data))
        myData = barcode.data.decode("utf-8")
        print("decoded data: "+str(myData))

        pts = np.array([barcode.polygon],np.int32)
        cv.polylines(frame,[pts],True,(255,0,0),5)
        pts2 = barcode.rect
        cv.putText(frame,myData,(pts2[0],pts2[1]),cv.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)
    cv.imshow('In',frame)
    break
    if cv.waitKey(1) & 0xFF == ord("e"):
        break

cv.waitKey(5000)
cv.destroyAllWindows()

#NOTE: in the barcode object we have
# - data: 
# - type:  
# - rect: tuple (left,top,width,height) left,top gives origin coordinates
# - polygon: tuple gives the coordinates of the points to draw around the qr code
# - quality: 1
# - orientation: 'UP'