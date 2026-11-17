import cv2
import numpy as np
import math
import serial
import time

ARDUINO_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 9600
CAMERA_INDEX = 0

SERVO_MIN = 0
SERVO_MAX = 180
SERVO_HOME = 90

KP = 40.0  
KD = 8.0   
SMOOTHING = 0.3
DEADZONE = 0.015  
MAX_ANGLE_CHANGE = 3  

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.10 


try:
    calib_data = np.load('camera_calibration.npz')
    camera_matrix = calib_data['camera_matrix']
    dist_coeffs = calib_data['dist_coeffs']
    print("✓ Camera calibration loaded")
except:
    print("ERROR: camera_calibration.npz not found!")
    exit()

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Connected to Arduino")
except Exception as e:
    print(f"ERROR: {e}")
    exit()

cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("ERROR: Cannot open camera")
    exit()

ret, frame = cap.read()
if ret:
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    print(f"Camera resolution: {frame_width}x{frame_height}")

current_servo_angle = SERVO_HOME
previous_error_3d = 0.0

def send_servo_angle(angle):
    """Send angle to Arduino"""
    angle = int(np.clip(angle, SERVO_MIN, SERVO_MAX))
    arduino.write(f"{angle}\n".encode())
    return angle

def calculate_servo_from_3d_position(x_3d, z_3d):
    """
    Calculate servo angle using angular error (geometrically correct)
    """
    global current_servo_angle, previous_error_3d
    
    angular_error = math.atan2(-x_3d, z_3d)  
    angular_error_deg = math.degrees(angular_error)
    
    ANGLE_DEADZONE = 2.0
    if abs(angular_error_deg) < ANGLE_DEADZONE:
        previous_error_3d = angular_error_deg
        return current_servo_angle
    
    KP_ANGLE = 0.8
    KD_ANGLE = 0.15
    
    p_term = angular_error_deg * KP_ANGLE
    derivative = angular_error_deg - previous_error_3d
    d_term = derivative * KD_ANGLE
    previous_error_3d = angular_error_deg
    
    angle_adjustment = p_term + d_term
    angle_adjustment = np.clip(angle_adjustment, -MAX_ANGLE_CHANGE, MAX_ANGLE_CHANGE)
    
    target_angle = current_servo_angle + angle_adjustment
    target_angle = np.clip(target_angle, SERVO_MIN, SERVO_MAX)
    
    smoothed_angle = SMOOTHING * current_servo_angle + (1 - SMOOTHING) * target_angle
    
    return smoothed_angle

print("\n" + "="*60)
print("3D Pose-Based ArUco Tracking")
print("="*60)
print("Using 3D X-position (meters) for control")
print("Target: X = 0.0m (marker directly in front of camera)")
print("Press 'q' to quit, 'h' to home servo")
print("="*60 + "\n")

send_servo_angle(SERVO_HOME)
time.sleep(1)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        corners, ids, rejected = detector.detectMarkers(frame)
        
        cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), 
                (0, 255, 0), 1)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, camera_matrix, dist_coeffs
            )
            
            marker_id = ids[0][0]
            
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                            rvecs[0], tvecs[0], MARKER_SIZE * 0.5)
            
            x_3d = tvecs[0][0][0]
            y_3d = tvecs[0][0][1]  
            z_3d = tvecs[0][0][2]
            
            marker_corners = corners[0][0]
            marker_center_x_px = int(np.mean(marker_corners[:, 0]))
            marker_center_y_px = int(np.mean(marker_corners[:, 1]))
            cv2.circle(frame, (marker_center_x_px, marker_center_y_px), 8, (0, 0, 255), -1)
            
            new_angle = calculate_servo_from_3d_position(x_3d, z_3d)
            
            if new_angle != current_servo_angle:
                current_servo_angle = send_servo_angle(new_angle)
            
            error_3d = 0.0 - x_3d
            
            cv2.putText(frame, f"Marker ID: {marker_id}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"3D Position:", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"  X: {x_3d:+.3f}m (L/R)", (10, 85), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"  Y: {y_3d:+.3f}m (U/D)", (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"  Z: {z_3d:.3f}m (dist)", (10, 135), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"3D Error: {error_3d:+.3f}m", (10, 165), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Servo: {current_servo_angle:.1f}", (10, 195), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            print(f"Marker {marker_id} | "
                  f"3D_X: {x_3d:+.3f}m | "
                  f"Z: {z_3d:.3f}m | "
                  f"Error: {error_3d:+.3f}m | "
                  f"Servo: {current_servo_angle:5.1f}°")
        else:
            cv2.putText(frame, "No marker detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"Servo: {current_servo_angle:.1f}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('3D Pose-Based Tracking', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('h'):
            print("Homing servo")
            current_servo_angle = send_servo_angle(SERVO_HOME)
            time.sleep(0.5)

except KeyboardInterrupt:
    print("\nInterrupted")

finally:
    print("\nReturning to home...")
    send_servo_angle(SERVO_HOME)
    time.sleep(1)
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
    print("Done")
