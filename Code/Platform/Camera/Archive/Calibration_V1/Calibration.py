import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5*5,3), np.float32)
objp[:,:2] = np.mgrid[0:5,0:5].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('visualisation_calibration/Calibration_pictures/*.jpg')
print("images nb "+str(len(images)))
x=0
for fname in images:
    print("image")
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (5,5),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (5,5), corners,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

        # It returns the camera matrix, distortion coefficients, rotation and translation vectors etc.
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        # Now we can take an image and undistort it.
        # But before that, we can refine the camera matrix based on a free scaling parameter.
        #   0 : undistorted image with minimum unwanted pixels. So it may even remove some pixels at image corners
        #   1 : all pixels are retained with some extra black images.
        # Also return a ROI to crop the image
        img = cv2.imread(fname)
        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

        # undistort
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        print(dst)
        cv2.imwrite("visualisation_calibration/Calibration_pictures/result_"+str(x)+'.png',dst)
        print(x)
        x+=1

cv2.destroyAllWindows()