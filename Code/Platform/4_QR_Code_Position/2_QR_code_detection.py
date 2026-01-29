import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode
from pyzbar.pyzbar import ZBarSymbol
 

#NOTE: Change these parameters in accordance to your hardware
#----------------------------------------------------
camera_ID = 1 # usually 0 is the builtin camera
camera_width = 2560 # Width for stereo camera (will be split in half)
camera_height = 720
#----------------------------------------------------
resize_scale = 0.6 #0.4 for 1440*3840

print("Connecting to Camera...")
cam = cv.VideoCapture(camera_ID)
# cam.set(cv.CAP_PROP_FPS, 60)
cam.set(cv.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, camera_height)

ret, frame = cam.read()
assert ret, "No frame readed"

last_picture_taken = frame
last_picture_taken = cv.resize(last_picture_taken, None, fx=resize_scale, fy=resize_scale)

height = frame.shape[0]
width = frame.shape[1] 
print("Picture width: "+str(width)+"     height: "+str(height))
print("Connected to camera")
cv.namedWindow("window",cv.WINDOW_AUTOSIZE)


while (True):
    key = cv.waitKey(1)
    ret = False
    while not(ret):
        ret,frame = cam.read()
    
    frame_live = cv.resize(frame, None, fx=resize_scale, fy=resize_scale)

    if key == 115: #s
        last_picture_taken = frame_live
        left=frame[0:height,0:int(width/2)]
        right=frame[0:height,int(width/2):(width)]


        # Detect QR codes using pyzbar
        qr_codes_left = decode(left)
        qr_codes_right = decode(right)
        

        # Display QR code information
        if qr_codes_left:
            print(f"\n--- QR Code(s) Detected: {len(qr_codes_left)} ---")
            for i, qr_code in enumerate(qr_codes_left):
                print(f"\nQR Code #{i+1}:")
                print(f"  Data: {qr_code.data.decode('utf-8')}")
                print(f"  Type: {qr_code.type}")
                print(f"  Location: {qr_code.rect}")
                print(f"  Quality: {qr_code.quality}")
    
    if key == 101: #e
        break

        
    imOut = np.vstack((frame_live,last_picture_taken))
    cv.imshow("window", imOut)

cam.release()
cv.destroyAllWindows()