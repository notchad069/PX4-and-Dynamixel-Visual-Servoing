#!/usr/bin/env python3

import cv2
import numpy as np
from dynamixel_sdk import *
import time


PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_VELOCITY = 104

PAN_SERVO_ID = 1
TILT_SERVO_ID = 2

PAN_MIN, PAN_MAX = 0, 4095
TILT_MIN, TILT_MAX = 0, 4095
PAN_CENTER = 2048
TILT_CENTER = 2048

DEG_TO_UNIT = 4096 / 360.0
RAD_TO_UNIT = 4096 / (2 * np.pi)

camera_matrix = np.array([
    [800.0, 0.0, 320.0],
    [0.0, 800.0, 240.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1))
MARKER_SIZE = 0.05

CONTROL_RATE = 0.05  


class ExponentialFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.filtered_value = None
    
    def update(self, measurement):
        if self.filtered_value is None:
            self.filtered_value = measurement
        else:
            self.filtered_value = self.alpha * measurement + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    def reset(self):
        self.filtered_value = None


class CubicTrajectoryGenerator:
 
    def __init__(self, duration=1.0):
        self.duration = duration  
        self.q_start = 0.0
        self.q_end = 0.0
        self.start_time = None
        self.active = False
    
    def new_trajectory(self, q_start, q_end, current_time):
        self.q_start = q_start
        self.q_end = q_end
        self.start_time = current_time
        self.active = True
    
    def get_setpoint(self, current_time):
        
        if not self.active:
            return None
        
        elapsed = current_time - self.start_time
        
        if elapsed >= self.duration:
            self.active = False
            return (self.q_end, 0.0)
        
        tau = elapsed / self.duration
        
       
        delta_q = self.q_end - self.q_start
        
        position = self.q_start + delta_q * (3*tau**2 - 2*tau**3)
        
        velocity = delta_q * (6*tau - 6*tau**2) / self.duration
        
        return (position, velocity)
    
    def is_active(self):
        return self.active
    

    

class PDController:
    def __init__(self, kp, kd, output_limit=None, derivative_filter_alpha=0.3):
        self.kp = kp
        self.kd = kd
        self.output_limit = output_limit
        self.derivative_filter_alpha = derivative_filter_alpha
        self.prev_error = 0.0
        self.prev_error_filtered = 0.0
        self.prev_time = time.time()
    
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 1e-6
        
        p_term = self.kp * error

        if self.kd > 0:
            if self.prev_error_filtered == 0.0:
                error_filtered = error
            else:
                error_filtered = (self.derivative_filter_alpha * error + 
                                (1 - self.derivative_filter_alpha) * self.prev_error_filtered)
            
            d_term = self.kd * (error_filtered - self.prev_error_filtered) / dt
            self.prev_error_filtered = error_filtered
        else:
            d_term = 0.0
        
        output = p_term + d_term
        
        if self.output_limit:
            output = np.clip(output, -self.output_limit, self.output_limit)
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.prev_error_filtered = 0.0
        self.prev_time = time.time()
class TrajectoryController:
    
    def __init__(self, kp, kd, velocity_gain=1.0):
        self.kp = kp
        self.kd = kd
        self.velocity_gain = velocity_gain  
        self.prev_error = 0.0
        self.prev_time = time.time()
    
    def compute(self, desired_pos, desired_vel, actual_pos):
        
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 1e-6
        
        error = desired_pos - actual_pos
        
        p_term = self.kp * error
        d_term = self.kd * (error - self.prev_error) / dt
        
        velocity_feedforward = self.velocity_gain * desired_vel
        
        control_velocity = velocity_feedforward + p_term + d_term
        
        self.prev_error = error
        self.prev_time = current_time
        
        return control_velocity
    
    def reset(self):
        self.prev_error = 0.0
        self.prev_time = time.time()

class DynamixelController:
    def __init__(self):
        global PAN_CENTER, TILT_CENTER, PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX
        
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        if not self.port_handler.openPort():
            raise Exception("Failed to open port")
        
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise Exception("Failed to set baudrate")
        
        print("Port opened successfully")
        
        self._enable_torque(PAN_SERVO_ID)
        self._enable_torque(TILT_SERVO_ID)
        
        print("\nReading current servo positions as home...")
        current_pan = self.get_pan_position()
        current_tilt = self.get_tilt_position()
        
        print(f"Pan current position: {current_pan}")
        print(f"Tilt current position: {current_tilt}")
        
        PAN_CENTER = current_pan
        TILT_CENTER = current_tilt
        
        self.commanded_pan = current_pan
        self.commanded_tilt = current_tilt
        
        PAN_MIN = max(0, current_pan - 1024)
        PAN_MAX = min(4095, current_pan + 1024)
        TILT_MIN = max(0, current_tilt - 1024)
        TILT_MAX = min(4095, current_tilt + 1024)
        
        print(f"New Pan center: {PAN_CENTER}, range: [{PAN_MIN}, {PAN_MAX}]")
        print(f"New Tilt center: {TILT_CENTER}, range: [{TILT_MIN}, {TILT_MAX}]")
        print("Servos ready at current position")
    
    def _enable_torque(self, servo_id):
        print(f"\n--- Setting up Servo {servo_id} ---")
        
        self.packet_handler.write1ByteTxRx(self.port_handler, servo_id, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.1)
        
        self.packet_handler.write1ByteTxRx(self.port_handler, servo_id, ADDR_OPERATING_MODE, 1)
        time.sleep(0.1)
        
        self.packet_handler.write1ByteTxRx(self.port_handler, servo_id, ADDR_TORQUE_ENABLE, 1)

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, servo_id, ADDR_TORQUE_ENABLE, 1)
        
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            raise Exception(f"Torque enable failed for servo {servo_id}")
        
        print(f"Servo {servo_id} ready!")
    
    def set_pan_position(self, position):
        position = int(np.clip(position, PAN_MIN, PAN_MAX))
        self.packet_handler.write4ByteTxRx(
            self.port_handler, PAN_SERVO_ID, ADDR_GOAL_POSITION, position)
        self.commanded_pan = position
    
    def set_tilt_position(self, position):
        position = int(np.clip(position, TILT_MIN, TILT_MAX))
        self.packet_handler.write4ByteTxRx(
            self.port_handler, TILT_SERVO_ID, ADDR_GOAL_POSITION, position)
        self.commanded_tilt = position
    def set_pan_velocity(self, velocity_rad_per_sec):
    
   
        rpm = velocity_rad_per_sec * 60 / (2 * np.pi)
        velocity_units = int(rpm / 0.229)
    
        velocity_units = int(np.clip(velocity_units, -500, 500))
    
        self.packet_handler.write4ByteTxRx(
            self.port_handler, PAN_SERVO_ID, ADDR_GOAL_VELOCITY, velocity_units)   
    def set_tilt_velocity(self, velocity_rad_per_sec):
        rpm = velocity_rad_per_sec * 60 / (2 * np.pi)
        velocity_units = int(rpm / 0.229)
        velocity_units = int(np.clip(velocity_units, -500, 500))
    
        self.packet_handler.write4ByteTxRx(
            self.port_handler, TILT_SERVO_ID, ADDR_GOAL_VELOCITY, velocity_units) 
    
    def get_pan_position(self):
        pos, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, PAN_SERVO_ID, ADDR_PRESENT_POSITION)
        return pos
    
    def get_tilt_position(self):
        pos, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, TILT_SERVO_ID, ADDR_PRESENT_POSITION)
        return pos
    
    def get_commanded_pan(self):
        return self.commanded_pan
    
    def get_commanded_tilt(self):
        return self.commanded_tilt
    
    def close(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, PAN_SERVO_ID, ADDR_TORQUE_ENABLE, 0)
        self.packet_handler.write1ByteTxRx(self.port_handler, TILT_SERVO_ID, ADDR_TORQUE_ENABLE, 0)
        self.port_handler.closePort()


def test_communication():
    print("\n=== Testing Servo Communication ===")
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    try:
        if not port_handler.openPort() or not port_handler.setBaudRate(BAUDRATE):
            return False
        
        print(f"Port: {DEVICENAME}, Baudrate: {BAUDRATE}\nScanning...")
        
        found = 0
        for servo_id in [1, 2]:
            model_num, comm_result, error = packet_handler.ping(port_handler, servo_id)
            if comm_result == COMM_SUCCESS:
                print(f"  ✓ Servo {servo_id}: Model {model_num}")
                found += 1
        
        port_handler.closePort()
        return found == 2
            
    except Exception as e:
        print(f"Test failed: {e}")
        return False


def main():
    if not test_communication():
        return
    
    print("Initializing camera...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return
    
    print("Initializing ArUco detector...")
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    print("\nInitializing Dynamixel controller...")
    try:
        dxl = DynamixelController()
    except Exception as e:
        print(f"Failed: {e}")
        cap.release()
        return
    
    pan_trajectory = CubicTrajectoryGenerator(duration=0.8) 
    tilt_trajectory = CubicTrajectoryGenerator(duration=0.8)
    
    pan_controller = TrajectoryController(kp=0.2, kd=0.035, velocity_gain=0.8)
    tilt_controller = TrajectoryController(kp=0.2, kd=0.035, velocity_gain=0.8)
    
    tvec_filter_x = ExponentialFilter(alpha=0.3)  
    tvec_filter_y = ExponentialFilter(alpha=0.3)
    tvec_filter_z = ExponentialFilter(alpha=0.3)
    
    ERROR_THRESHOLD = 0.09  
    TRAJECTORY_COOLDOWN = 0.5  
    
    last_pan_trajectory_time = 0
    last_tilt_trajectory_time = 0
    
    print("\n=== Velocity Control with Cubic Trajectories ===")
    print("Trajectory duration: 800ms")
    print("Error threshold: 4.5 degrees")
    print("Press 'q' to quit\n")
    
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            current_time = time.time()
            
            corners, ids, rejected = detector.detectMarkers(frame)
            
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE, camera_matrix, dist_coeffs)
                
                tvec_raw = tvecs[0][0]
                rvec = rvecs[0][0]
                
                tx_smooth = tvec_filter_x.update(tvec_raw[0])
                ty_smooth = tvec_filter_y.update(tvec_raw[1])
                tz_smooth = tvec_filter_z.update(tvec_raw[2])
                
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                 rvec, tvec_raw, MARKER_SIZE * 0.5)
                
                pan_error = np.arctan2(tx_smooth, tz_smooth)
                tilt_error = np.arctan2(ty_smooth, tz_smooth)
                
                current_pan = dxl.get_pan_position()
                current_tilt = dxl.get_tilt_position()
                
                current_pan_rad = (current_pan - PAN_CENTER) / RAD_TO_UNIT
                current_tilt_rad = (current_tilt - TILT_CENTER) / RAD_TO_UNIT
                
                
                time_since_last_pan = current_time - last_pan_trajectory_time
                
                if (abs(pan_error) > ERROR_THRESHOLD and 
                    not pan_trajectory.is_active() and 
                    time_since_last_pan > TRAJECTORY_COOLDOWN):
                    
                    if (current_pan_rad > 0 and pan_error > 0) or (current_pan_rad < 0 and pan_error < 0):
                        target_pan_rad = current_pan_rad - pan_error
                    else:
                        target_pan_rad = current_pan_rad + pan_error
                    
                    pan_trajectory.new_trajectory(current_pan_rad, target_pan_rad, current_time)
                    last_pan_trajectory_time = current_time
                    
                    print(f"New PAN trajectory: {current_pan_rad:.3f} -> {target_pan_rad:.3f} rad (error: {np.degrees(pan_error):.1f}°)")
                
               
                pan_setpoint = pan_trajectory.get_setpoint(current_time)
                
                if pan_setpoint is not None:
                    target_pan_pos, target_pan_vel = pan_setpoint
                    
                    
                    current_pan_feedback = dxl.get_pan_position()
                    current_pan_rad_feedback = (current_pan_feedback - PAN_CENTER) / RAD_TO_UNIT
                    
                    pan_velocity = pan_controller.compute(target_pan_pos, target_pan_vel, current_pan_rad_feedback)
                else:
                    
                    if abs(pan_error) > 0.02:  
                        pan_velocity = -1.5 * pan_error  
                    else:
                        pan_velocity = 0.0
                
                time_since_last_tilt = current_time - last_tilt_trajectory_time
                
                if (abs(tilt_error) > ERROR_THRESHOLD and 
                    not tilt_trajectory.is_active() and 
                    time_since_last_tilt > TRAJECTORY_COOLDOWN):
                    if tilt_error > 0:
                        target_tilt_rad = current_tilt_rad - tilt_error
                    else:
                        target_tilt_rad = current_tilt_rad + tilt_error

                    tilt_trajectory.new_trajectory(current_tilt_rad, target_tilt_rad, current_time)
                    last_tilt_trajectory_time = current_time
                    
                    print(f"New TILT trajectory: {current_tilt_rad:.3f} -> {target_tilt_rad:.3f} rad (error: {np.degrees(tilt_error):.1f}°)")
                
                tilt_setpoint = tilt_trajectory.get_setpoint(current_time)
                
                if tilt_setpoint is not None:
                    target_tilt_pos, target_tilt_vel = tilt_setpoint
                    
                    current_tilt_feedback = dxl.get_tilt_position()
                    current_tilt_rad_feedback = (current_tilt_feedback - TILT_CENTER) / RAD_TO_UNIT
                    
                    tilt_velocity = tilt_controller.compute(target_tilt_pos, target_tilt_vel, current_tilt_rad_feedback)
                else:
                    if abs(tilt_error) > 0.02:
                        tilt_velocity = 1.5 * tilt_error
                    else:
                        tilt_velocity = 0.0
                
                dxl.set_pan_velocity(pan_velocity)
                dxl.set_tilt_velocity(tilt_velocity)
                
                if frame_count % 30 == 0:
                    pan_active = "ACTIVE" if pan_trajectory.is_active() else "IDLE"
                    tilt_active = "ACTIVE" if tilt_trajectory.is_active() else "IDLE"
                    print(f"Pan: {np.degrees(pan_error):.1f}° [{pan_active}] v={pan_velocity:.2f}, Tilt: {np.degrees(tilt_error):.1f}° [{tilt_active}] v={tilt_velocity:.2f}")
                
                cv2.putText(frame, f"Pan Error: {np.degrees(pan_error):.2f} deg", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Tilt Error: {np.degrees(tilt_error):.2f} deg", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Distance: {tz_smooth:.2f} m", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                pan_status = "MOVING" if pan_trajectory.is_active() else "HOLDING"
                tilt_status = "MOVING" if tilt_trajectory.is_active() else "HOLDING"
                cv2.putText(frame, f"Pan: {pan_status}, Tilt: {tilt_status}", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
            else:
                pan_controller.reset()
                tilt_controller.reset()
                tvec_filter_x.reset()
                tvec_filter_y.reset()
                tvec_filter_z.reset()
                
                dxl.set_pan_velocity(0.0)
                dxl.set_tilt_velocity(0.0)
                
                cv2.putText(frame, "No marker - STOPPED", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.imshow("ArUco Tracking - Velocity Control", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        print("\nShutting down...")
        dxl.set_pan_velocity(0.0)
        dxl.set_tilt_velocity(0.0)
        time.sleep(0.1)
        
        cap.release()
        cv2.destroyAllWindows()
        dxl.close()
        print("Done!")


if __name__ == "__main__":
    main()