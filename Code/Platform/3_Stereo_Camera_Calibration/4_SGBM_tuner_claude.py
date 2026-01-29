# SGBM Live Tuner for Stereo Camera
# This code allows real-time adjustment of SGBM parameters using trackbars
# Use this to find optimal parameters for your stereo depth estimation

import numpy as np
import cv2 as cv

#NOTE: Change these parameters according to your hardware
#----------------------------------------------------
camera_ID = 1  # Usually 0 is the builtin camera
camera_width = 2560  # Width for stereo camera (will be split in half)
camera_height = 720
#----------------------------------------------------
resize_scale = 0.5  # Scale for display

# Load rectification parameters if available
try:
    calibration = np.load("Saved_parameters/retification_informations.npz")
    leftMapX = calibration['leftMapX']
    leftMapY = calibration['leftMapY']
    rightMapX = calibration['rightMapX']
    rightMapY = calibration['rightMapY']
    use_rectification = True
    print("Loaded rectification parameters")
except:
    use_rectification = False
    print("No rectification parameters found, using raw images")

# Connect to camera
print("Connecting to camera...")
cam = cv.VideoCapture(camera_ID)
cam.set(cv.CAP_PROP_FRAME_WIDTH, camera_width)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, camera_height)

ret, frame = cam.read()
assert ret, "Failed to read from camera"
print("Camera connected successfully")

height = frame.shape[0]
width = frame.shape[1]

# Create windows
cv.namedWindow('Disparity Map', cv.WINDOW_AUTOSIZE)
cv.namedWindow('Controls', cv.WINDOW_NORMAL)
cv.resizeWindow('Controls', 600, 800)

# Global variables for SGBM parameters
params = {
    'minDisparity': 0,
    'numDisparities': 64,
    'blockSize': 5,
    'P1': 8,
    'P2': 32,
    'disp12MaxDiff': 1,
    'preFilterCap': 63,
    'uniquenessRatio': 10,
    'speckleWindowSize': 100,
    'speckleRange': 32,
    'mode': 0  # 0: SGBM, 1: HH (3-way)
}

def update_disparity(val=0):
    """Callback function for trackbar updates"""
    pass

# Create trackbars for SGBM parameters
cv.createTrackbar('minDisparity', 'Controls', params['minDisparity'], 25, update_disparity)
cv.createTrackbar('numDisparities', 'Controls', 4, 16, update_disparity)  # Will be multiplied by 16
cv.createTrackbar('blockSize', 'Controls', params['blockSize'], 21, update_disparity)  # Must be odd
cv.createTrackbar('P1 (x8*block)', 'Controls', 1, 50, update_disparity)  # Will be multiplied by 8*blockSize*blockSize
cv.createTrackbar('P2 (x32*block)', 'Controls', 1, 50, update_disparity)  # Will be multiplied by 32*blockSize*blockSize
cv.createTrackbar('disp12MaxDiff', 'Controls', params['disp12MaxDiff'], 25, update_disparity)
cv.createTrackbar('preFilterCap', 'Controls', params['preFilterCap'], 63, update_disparity)
cv.createTrackbar('uniquenessRatio', 'Controls', params['uniquenessRatio'], 100, update_disparity)
cv.createTrackbar('speckleWindowSize', 'Controls', params['speckleWindowSize'], 200, update_disparity)
cv.createTrackbar('speckleRange', 'Controls', params['speckleRange'], 100, update_disparity)
cv.createTrackbar('mode', 'Controls', params['mode'], 1, update_disparity)  # 0: SGBM, 1: HH

# Set initial trackbar positions
cv.setTrackbarPos('numDisparities', 'Controls', 4)  # 4*16 = 64
cv.setTrackbarPos('blockSize', 'Controls', 5)

print("\nControls:")
print("- Adjust trackbars to tune SGBM parameters")
print("- Press 'p' to print current parameters")
print("- Press 's' to save current parameters")
print("- Press 'e' to exit")
print("\nParameter Guidelines:")
print("- numDisparities: Must be divisible by 16 (tracked value * 16)")
print("- blockSize: Must be odd, between 3-21")
print("- P1, P2: Smoothness parameters (P2 > P1)")
print("- uniquenessRatio: 5-15 typically good")
print("- speckleWindowSize: 50-200 for noise filtering")
print("- mode: 0=SGBM (faster), 1=HH (better quality)")

while True:
    key = cv.waitKey(1)
    
    # Read frame from camera
    ret = False
    while not ret:
        ret, frame = cam.read()
    
    # Crop frame into left and right pictures
    left=frame[0:height,0:int(width/2)]
    right=frame[0:height,int(width/2):(width)]
    
    # Apply rectification if available
    if use_rectification:
        left = cv.remap(left, leftMapX, leftMapY, cv.INTER_LINEAR)
        right = cv.remap(right, rightMapX, rightMapY, cv.INTER_LINEAR)
    
    # Convert to grayscale
    gray_left = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
    gray_right = cv.cvtColor(right, cv.COLOR_BGR2GRAY)
    
    # Get current trackbar values
    minDisparity = cv.getTrackbarPos('minDisparity', 'Controls')
    numDisparities = cv.getTrackbarPos('numDisparities', 'Controls') * 16
    blockSize = cv.getTrackbarPos('blockSize', 'Controls')
    P1_mult = cv.getTrackbarPos('P1 (x8*block)', 'Controls')
    P2_mult = cv.getTrackbarPos('P2 (x32*block)', 'Controls')
    disp12MaxDiff = cv.getTrackbarPos('disp12MaxDiff', 'Controls')
    preFilterCap = cv.getTrackbarPos('preFilterCap', 'Controls')
    uniquenessRatio = cv.getTrackbarPos('uniquenessRatio', 'Controls')
    speckleWindowSize = cv.getTrackbarPos('speckleWindowSize', 'Controls')
    speckleRange = cv.getTrackbarPos('speckleRange', 'Controls')
    mode = cv.getTrackbarPos('mode', 'Controls')
    
    # Ensure valid values
    if numDisparities < 16:
        numDisparities = 16
    if blockSize < 3:
        blockSize = 3
    if blockSize % 2 == 0:  # Must be odd
        blockSize += 1
    if blockSize > 21:
        blockSize = 21
    
    # Calculate P1 and P2 based on blockSize
    P1 = P1_mult * 8 * blockSize * blockSize
    P2 = P2_mult * 32 * blockSize * blockSize
    
    # Create SGBM object with current parameters
    if mode == 0:
        stereo = cv.StereoSGBM_create(
            minDisparity=minDisparity,
            numDisparities=numDisparities,
            blockSize=blockSize,
            P1=P1,
            P2=P2,
            disp12MaxDiff=disp12MaxDiff,
            preFilterCap=preFilterCap,
            uniquenessRatio=uniquenessRatio,
            speckleWindowSize=speckleWindowSize,
            speckleRange=speckleRange,
            mode=cv.STEREO_SGBM_MODE_SGBM
        )
    else:  # HH mode (3-way)
        stereo = cv.StereoSGBM_create(
            minDisparity=minDisparity,
            numDisparities=numDisparities,
            blockSize=blockSize,
            P1=P1,
            P2=P2,
            disp12MaxDiff=disp12MaxDiff,
            preFilterCap=preFilterCap,
            uniquenessRatio=uniquenessRatio,
            speckleWindowSize=speckleWindowSize,
            speckleRange=speckleRange,
            mode=cv.STEREO_SGBM_MODE_HH
        )
    
    # Compute disparity map
    disparity = stereo.compute(gray_left, gray_right)
    
    # Normalize disparity for visualization
    disparity_normalized = cv.normalize(disparity, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
    
    # Apply colormap for better visualization
    disparity_color = cv.applyColorMap(disparity_normalized, cv.COLORMAP_JET)
    
    # Resize for display
    display_left = cv.resize(left, None, fx=resize_scale, fy=resize_scale)
    display_disparity = cv.resize(disparity_color, None, fx=resize_scale, fy=resize_scale)
    
    # Stack images for comparison
    display_output = np.hstack((display_left, display_disparity))
    
    # Add parameter text overlay
    text_lines = [
        f"minDisp:{minDisparity} numDisp:{numDisparities} block:{blockSize}",
        f"P1:{P1} P2:{P2} disp12:{disp12MaxDiff}",
        f"preFilt:{preFilterCap} unique:{uniquenessRatio}",
        f"speckWin:{speckleWindowSize} speckRng:{speckleRange}",
        f"mode:{'SGBM' if mode==0 else 'HH'}"
    ]
    
    y_offset = 30
    for i, line in enumerate(text_lines):
        cv.putText(display_output, line, (10, y_offset + i*25), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    cv.imshow('Disparity Map', display_output)
    
    # Handle key presses
    if key == 112:  # 'p' - print parameters
        print("\n=== Current SGBM Parameters ===")
        print(f"minDisparity: {minDisparity}")
        print(f"numDisparities: {numDisparities}")
        print(f"blockSize: {blockSize}")
        print(f"P1: {P1}")
        print(f"P2: {P2}")
        print(f"disp12MaxDiff: {disp12MaxDiff}")
        print(f"preFilterCap: {preFilterCap}")
        print(f"uniquenessRatio: {uniquenessRatio}")
        print(f"speckleWindowSize: {speckleWindowSize}")
        print(f"speckleRange: {speckleRange}")
        print(f"mode: {'SGBM' if mode==0 else 'HH'}")
        print("=" * 35)
    
    elif key == 115:  # 's' - save parameters
        params_dict = {
            'minDisparity': minDisparity,
            'numDisparities': numDisparities,
            'blockSize': blockSize,
            'P1': P1,
            'P2': P2,
            'disp12MaxDiff': disp12MaxDiff,
            'preFilterCap': preFilterCap,
            'uniquenessRatio': uniquenessRatio,
            'speckleWindowSize': speckleWindowSize,
            'speckleRange': speckleRange,
            'mode': mode
        }
        np.savez_compressed("Saved_parameters/sgbm_parameters.npz", **params_dict)
        print("\nParameters saved to Saved_parameters/sgbm_parameters.npz")
    
    elif key == 101:  # 'e' - exit
        break

cam.release()
cv.destroyAllWindows()
print("\nSGBM tuner closed")
