import cv2
import numpy as np
import serial
import time

ARDUINO_PORT = '/dev/ttyACM0'  
BAUD_RATE = 9600
CAMERA_INDEX = 0
HOME_POSITION = 90  
ARUCO_DICT = cv2.aruco.DICT_4X4_50  

KP = 0.1  
KI = 0.0 
KD = 0.8

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  
    print(f"Connected to Arduino on {ARDUINO_PORT}")
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    print("Please check the port name and try again")
    exit()

cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

previous_error = 0
integral = 0
current_servo_angle = HOME_POSITION

arduino.write(f"{HOME_POSITION}\n".encode())

print("\nAruCo Marker Tracking Started")
print("Press 'q' to quit")
print("Press 'h' to return to home position")
print("\n")

def calculate_target_angle(marker_center_x, frame_width):
    """
    Calculate the target servo angle based on marker position
    Args:
        marker_center_x: X coordinate of marker center
        frame_width: Width of the video frame
    Returns:
        Target angle for servo (0-180 degrees)
    """
    frame_center = frame_width / 2
    error = marker_center_x - frame_center
    
    error_normalized = error / frame_center
    
    angle_offset = -error_normalized * 90  
    target_angle = HOME_POSITION + angle_offset
    
    target_angle = max(0, min(180, target_angle))

    deadband = 7
    if abs(target_angle - current_servo_angle) < deadband:
        target_angle = current_servo_angle
    
    return target_angle, error

def pid_control(error, dt=0.033):
    """
    PID controller for smoother servo movement
    """
    global previous_error, integral
    
    P = KP * error
    
    integral += error * dt
    I = KI * integral
    
    derivative = (error - previous_error) / dt
    D = KD * derivative
    
    previous_error = error
    
    return P + I + D
    
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame")
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = detector.detectMarkers(gray)
        
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            marker_corners = corners[0][0]
            
            marker_center_x = int(np.mean(marker_corners[:, 0]))
            marker_center_y = int(np.mean(marker_corners[:, 1]))
            
            cv2.circle(frame, (marker_center_x, marker_center_y), 5, (0, 255, 0), -1)
            
            frame_height, frame_width = frame.shape[:2]
            target_angle, error = calculate_target_angle(marker_center_x, frame_width)
            
            pid_correction = pid_control(error)
            target_angle = HOME_POSITION - pid_correction
            target_angle = max(0, min(180, target_angle))
            
            current_servo_angle = int(target_angle)
            arduino.write(f"{current_servo_angle}\n".encode())
            
            cv2.putText(frame, f"Target Angle: {current_servo_angle}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Error: {int(error)}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        else:
            if current_servo_angle != HOME_POSITION:
                cv2.putText(frame, "No marker detected - Returning home", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        frame_height, frame_width = frame.shape[:2]
        cv2.line(frame, (frame_width // 2, 0), (frame_width // 2, frame_height), 
                (255, 0, 0), 2)
        
        cv2.putText(frame, f"Servo: {current_servo_angle} deg", 
                   (10, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        cv2.imshow('AruCo Marker Tracking', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('h'):
            current_servo_angle = HOME_POSITION
            arduino.write(f"{HOME_POSITION}\n".encode())
            print("Returning to home position")
            integral = 0  
            previous_error = 0

except KeyboardInterrupt:
    print("\nTracking stopped by user")

finally:
    arduino.write(f"{HOME_POSITION}\n".encode())  
    time.sleep(0.5)
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
    print("Cleanup complete. Goodbye!")
