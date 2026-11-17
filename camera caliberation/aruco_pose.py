import cv2
import numpy as np
import math

# Load camera calibration
calib_data = np.load('camera_calibration.npz')
camera_matrix = calib_data['camera_matrix']
dist_coeffs = calib_data['dist_coeffs']

print("Camera calibration loaded!")
print("Camera Matrix:")
print(camera_matrix)

# ArUco setup - DICT_4X4_50 for your marker
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Marker size in meters (10cm = 0.10m, adjust if needed)
marker_size = 0.10

# Function to convert rotation vector to Euler angles (roll, pitch, yaw)
def rotation_vector_to_euler_angles(rvec):
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    
    # Extract Euler angles from rotation matrix
    # This uses the convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    sy = math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = 0
    
    # Convert from radians to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    return roll, pitch, yaw

# Open webcam (adjust index if needed)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Trying camera index 1...")
    cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("ERROR: Cannot open camera!")
    exit()

print("\n" + "="*60)
print("ArUco Pose Estimation Started!")
print("="*60)
print("Press 'q' to quit")
print("="*60 + "\n")

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break
    
    # Detect ArUco markers
    corners, ids, rejected = detector.detectMarkers(frame)
    
    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estimate pose for each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, dist_coeffs
        )
        
        for i in range(len(ids)):
            # Draw 3D axis on the marker (X=red, Y=green, Z=blue)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                            rvecs[i], tvecs[i], marker_size * 0.5)
            
            # Get position (translation vector)
            x, y, z = tvecs[i][0]
            
            # Get orientation (convert rotation vector to Euler angles)
            roll, pitch, yaw = rotation_vector_to_euler_angles(rvecs[i])
            
            # Display info on frame
            marker_id = ids[i][0]
            text_y = 30 + i * 150
            
            cv2.putText(frame, f"ID: {marker_id}", (10, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Position (m):", (10, text_y + 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"  X: {x:.3f}", (10, text_y + 45), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"  Y: {y:.3f}", (10, text_y + 65), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"  Z: {z:.3f}", (10, text_y + 85), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"Orientation (deg):", (10, text_y + 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"  Roll:  {roll:.1f}", (10, text_y + 130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"  Pitch: {pitch:.1f}", (10, text_y + 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"  Yaw:   {yaw:.1f}", (10, text_y + 170), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Print to console
            print(f"Marker {marker_id} | Pos: X={x:.3f}m Y={y:.3f}m Z={z:.3f}m | "
                  f"Orient: Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°")
    
    cv2.imshow('ArUco 6DOF Pose Estimation', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\nTracking stopped.")
