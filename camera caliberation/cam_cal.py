import numpy as np
import cv2
import glob

# IMPORTANT: Set this to match YOUR checkerboard
# Count the INNER corners (where black squares meet)
# Common: (9,6) means 10x7 squares, 9x6 inner corners
CHECKERBOARD = (8, 6)  

# Measure one square on your displayed checkerboard in mm
# If it's on a screen, you can estimate or set to 1.0 (relative units work too)
square_size = 1.0  # You can use 1.0 for now, actual size matters less for pose estimation

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store points from all images
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

# Load all captured images
images = glob.glob('calibration_images/*.jpg')

if len(images) == 0:
    print("ERROR: No images found in calibration_images/ folder!")
    exit()

print(f"Found {len(images)} images. Processing...\n")

img_shape = None
successful = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    if img_shape is None:
        img_shape = gray.shape[::-1]
    
    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret:
        objpoints.append(objp)
        
        # Refine corner locations to sub-pixel accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        
        # Visualize (optional, slows things down)
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Found Corners', img)
        cv2.waitKey(100)
        
        successful += 1
        print(f"✓ {fname} - Corners found ({successful}/{len(images)})")
    else:
        print(f"✗ {fname} - No corners detected")

cv2.destroyAllWindows()

if len(objpoints) == 0:
    print("\nERROR: No valid checkerboard patterns detected!")
    print("Make sure:")
    print("  1. CHECKERBOARD size matches your pattern")
    print("  2. Entire checkerboard is visible in images")
    print("  3. Images are clear (not blurry)")
    exit()

print(f"\n{'='*50}")
print(f"Calibrating camera using {len(objpoints)} valid images...")
print(f"{'='*50}\n")

# The actual calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

print("✓ Calibration complete!\n")
print("Camera Matrix:")
print(camera_matrix)
print("\nDistortion Coefficients:")
print(dist_coeffs)
print(f"\nReprojection Error: {ret:.4f} pixels")

if ret < 1.0:
    print("  → Excellent calibration!")
elif ret < 2.0:
    print("  → Good calibration")
else:
    print("  → Calibration might need improvement (try more/better images)")

# Save the calibration
np.savez('camera_calibration.npz', 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs)

print("\n✓ Saved calibration to 'camera_calibration.npz'")
print("\nYou can now use this for ArUco pose estimation!")
