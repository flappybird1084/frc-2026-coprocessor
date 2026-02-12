import numpy as np
import cv2

CHECKERBOARD = (9, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = [] 
imgpoints = [] 
img_size = None # Store the frame size here

cap = cv2.VideoCapture(0)
print("Press SPACE to capture frame, ESC to finish and calculate.")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    if img_size is None:
        img_size = frame.shape[:2][::-1] # (width, height)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret_corners:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret_corners)

    cv2.imshow('Calibration - Press Space to Capture', frame)
    
    key = cv2.waitKey(1)
    if key == 32: # Space Bar
        if ret_corners:
            print(f"Frame {len(objpoints) + 1} Captured!")
            objpoints.append(objp)
            imgpoints.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria))
        else:
            print("No checkerboard detected in this frame! Move it around.")
    elif key == 27: # ESC
        break

cap.release()
cv2.destroyAllWindows()

# Safety check: did you actually hit space?
if len(objpoints) > 0:
    print(f"Calculating using {len(objpoints)} frames... hang tight.")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    print("\n--- YOUR CALIBRATION RESULTS ---")
    print(f"fx: {mtx[0][0]}")
    print(f"fy: {mtx[1][1]}")
    print(f"cx: {mtx[0][2]}")
    print(f"cy: {mtx[1][2]}")
    print("\nDistortion Coefficients (dist):")
    print(dist)
else:
    print("Error: No frames were captured. You have to press SPACE while the board is visible!")