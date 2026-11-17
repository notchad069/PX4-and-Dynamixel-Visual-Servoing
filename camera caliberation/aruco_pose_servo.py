import cv2
import numpy as np
import math
import serial
import time

ARDUINO_PORT = '/dev/ttyUSB0'  
# ARDUINO_PORT = 'COM3'  
# ARDUINO_PORT = '/dev/cu.usbmodem14101'  
BAUD_RATE = 9600

# Camera configuration
CAMERA_INDEX = 0

SERVO_MIN = 0
SERVO_MAX = 180
SERVO_HOME = 90

KP = 0.3  
KD = 0.1
SMOOTHING = 0.3
DEADZONE = 15  
MAX_ANGLE_CHANGE = 3

previous_error = 0

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

# Setup ArUco detector
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Setup serial connection to Arduino
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print("✓ Connected to Arduino")
except Exception as e:
    print(f"ERROR: Could not connect to Arduino: {e}")
    print("Make sure:")
    print("  1. Arduino is plugged in")
    print("  2. Correct port is specified")
    print("  3. Arduino IDE Serial Monitor is closed")
    exit()

# Open camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("ERROR: Cannot open camera")
    exit()

# Get frame dimensions
ret, frame = cap.read()
if ret:
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    print(f"✓ Camera resolution: {frame_width}x{frame_height}")
    print(f"✓ Frame center X: {frame_center_x}")
else:
    print("ERROR: Cannot read from camera")
    exit()

# Current servo angle
current_servo_angle = SERVO_HOME

def send_servo_angle(angle):
    """Send angle command to Arduino"""
    angle = int(np.clip(angle, SERVO_MIN, SERVO_MAX))
    command = f"{angle}\n"
    arduino.write(command.encode())
    print(f"Sending angle: {angle}")
    return angle

def calculate_servo_adjustment(marker_x, frame_center_x, frame_width):
    """
    Calculate servo angle adjustment based on marker position
    Returns: new servo angle
    """
    global current_servo_angle,previous_error
    
    
    error_x = marker_x - frame_center_x
    
    if abs(error_x) < DEADZONE:
        previous_error = error_x
        return current_servo_angle
    
    max_error = frame_width / 2
    normalized_error = error_x / max_error  # -1 to 1
    p_term = normalized_error*90*KP
    
    derivative = error_x - previous_error
    d_term = derivative*KD
    previous_error = error_x
    
    # Calculate angle change (using proportional control)
    angle_adjustment = -(p_term + d_term)  # Negative because servo convention
    
    angle_adjustment = np.clip(angle_adjustment,-MAX_ANGLE_CHANGE,MAX_ANGLE_CHANGE)	
    target_angle = current_servo_angle + angle_adjustment
    target_angle = np.clip(target_angle,SERVO_MIN,SERVO_MAX)
    smoothed_angle = SMOOTHING*current_servo_angle + (1-SMOOTHING)*target_angle
    
    return smoothed_angle

print("\n" + "="*60)
print("ArUco Servo Tracking Started!")
print("="*60)
print("Press 'q' to quit")
print("Press 'h' to home servo (90°)")
print("="*60 + "\n")

# Initialize servo to home position
send_servo_angle(SERVO_HOME)
time.sleep(1)

try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
            break
        
        # Detect ArUco markers
        corners, ids, rejected = detector.detectMarkers(frame)
        
        # Draw center line for reference
        cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame_height), 
                (0, 255, 0), 1)
        cv2.circle(frame, (frame_center_x, frame_height//2), 5, (0, 255, 0), -1)
        
        # Draw deadzone
        deadzone_left = frame_center_x - DEADZONE
        deadzone_right = frame_center_x + DEADZONE
        cv2.line(frame, (deadzone_left, 0), (deadzone_left, frame_height), 
                (0, 255, 255), 1)
        cv2.line(frame, (deadzone_right, 0), (deadzone_right, frame_height), 
                (0, 255, 255), 1)
        
        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimate pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, camera_matrix, dist_coeffs
            )
            
            # Process first detected marker (you can extend to multiple)
            marker_id = ids[0][0]
            
            # Draw 3D axis
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                            rvecs[0], tvecs[0], MARKER_SIZE * 0.5)
            
            # Get marker center in image coordinates
            marker_corners = corners[0][0]
            marker_center_x = int(np.mean(marker_corners[:, 0]))
            marker_center_y = int(np.mean(marker_corners[:, 1]))
            
            # Draw marker center
            cv2.circle(frame, (marker_center_x, marker_center_y), 8, (0, 0, 255), -1)
            
            # Calculate error
            error_x = marker_center_x - frame_center_x
            
            # Calculate and send new servo angle
            new_angle = calculate_servo_adjustment(marker_center_x, frame_center_x, frame_width)
            
            if new_angle != current_servo_angle:
                current_servo_angle = send_servo_angle(new_angle)
            
            # Get 3D position
            x, y, z = tvecs[0][0]
            
            # Display info
            cv2.putText(frame, f"Marker ID: {marker_id}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Error X: {error_x:.0f} px", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Servo Angle: {current_servo_angle:.0f}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Distance: {z:.2f}m", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            print(f"Marker {marker_id} | X_err: {error_x:6.0f}px | "
                  f"Servo: {current_servo_angle:5.1f}° | Dist: {z:.2f}m")
        else:
            # No marker detected
            cv2.putText(frame, "No marker detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"Servo Angle: {current_servo_angle:.0f}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('ArUco Servo Tracking', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('h'):
            print("Homing servo to 90°")
            current_servo_angle = send_servo_angle(SERVO_HOME)
            time.sleep(0.5)

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    # Cleanup
    print("\nReturning servo to home position...")
    send_servo_angle(SERVO_HOME)
    time.sleep(1)
    
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
    print("✓ Cleanup complete")
