import cv2

for i in range(10):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # Windows
    if cap.isOpened():
        print(f"Camera found at index {i}")
        cap.release()