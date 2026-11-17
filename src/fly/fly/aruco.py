#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from enum import Enum
import math

class DroneState(Enum):
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT_SEARCH = 3
    SCANNING_360 = 4
    MARKER_TRACKING = 5
    HOVERING = 6

class QuinticTrajectory:
    
    def __init__(self, s0, sf, v0, vf, a0, af, duration):
        
        self.duration = duration
        T = duration
        
        
        self.coeffs = np.array([
            s0,
            v0,
            a0 / 2.0,
            (20*sf - 20*s0 - (8*vf + 12*v0)*T - (3*a0 - af)*T**2) / (2*T**3),
            (30*s0 - 30*sf + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T**2) / (2*T**4),
            (12*sf - 12*s0 - (6*vf + 6*v0)*T - (a0 - af)*T**2) / (2*T**5)
        ])
        
    def position(self, t):
        t = np.clip(t, 0, self.duration)
        powers = np.array([1, t, t**2, t**3, t**4, t**5])
        return np.dot(self.coeffs, powers)
    
    def velocity(self, t):
        t = np.clip(t, 0, self.duration)
        if t == 0:
            return self.coeffs[1]
        powers = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
        return np.dot(self.coeffs, powers)
    
    def acceleration(self, t):
        t = np.clip(t, 0, self.duration)
        if t == 0:
            return 2*self.coeffs[2]
        powers = np.array([0, 0, 2, 6*t, 12*t**2, 20*t**3])
        return np.dot(self.coeffs, powers)
    
    def is_finished(self, t):
        return t >= self.duration

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.prev_error = 0.0
        self.integral = 0.0
        
    def compute(self, error, dt):
        p_term = self.kp * error
        
        self.integral += error * dt
        if self.output_limit:
            self.integral = np.clip(self.integral, -self.output_limit, self.output_limit)
        i_term = self.ki * self.integral
        
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        output = p_term + i_term + d_term
        
        if self.output_limit:
            output = np.clip(output, -self.output_limit, self.output_limit)
            
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class ArucoMarkerTracker(Node):
    def __init__(self):
        super().__init__('aruco_marker_tracker')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        
        self.camera_matrix = np.array([
            [386.0, 0.0, 320.0],
            [0.0, 386.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)
        self.marker_size = 0.2  
        
        self.bridge = CvBridge()
        
        self.state = DroneState.ARMING
        self.marker_detected = False
        self.marker_position_cam = None  
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0
        self.current_waypoint_idx = 0
        self.armed = False
        self.offboard_enabled = False
        
        self.scan_start_yaw = 0.0
        self.scan_current_yaw = 0.0
        self.scan_target_position = np.array([0.0, 0.0, 0.0])
        self.scan_yaw_rate = 0.5  
        
        self.waypoints = [
            np.array([0.0, 0.0, -3.0]),    
            np.array([2.0, 0.0, -3.0]),    
            np.array([3.0, 1.5, -3.0]),    
            np.array([3.0, -1.5, -3.0]),   
            np.array([4.0, 0.0, -3.5]),    
        ]
        
        self.trajectory = None
        self.trajectory_start_time = None
        
        self.pid_x = PIDController(kp=0.3, ki=0.06, kd=0.1, output_limit=0.5)
        self.pid_y = PIDController(kp=0.3, ki=0.06, kd=0.1, output_limit=0.5)
        self.pid_z = PIDController(kp=0.3, ki=0.06, kd=0.1, output_limit=0.7)
        
        self.smoothed_vel = np.zeros(3)

        self.target_distance = 2.0  
        self.last_time = self.get_clock().now()
        self.state_timer = 0
        
        self.marker_lost_time = 0.0
        self.marker_lost_threshold = 2.0
        
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/realsense/color/image_raw',
            self.camera_callback,
            camera_qos
        )
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco Marker Tracker with Quintic Trajectories')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Camera topic: /realsense/color/image_raw')
        self.get_logger().info('Marker position (world): (5, 0, 2.5)')
        self.get_logger().info('Target: Hover 2m in front of marker')
        self.get_logger().info('=' * 60)
        
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            debug = cv_image.copy()
            
            if ids is not None and len(ids) > 0:
                self.marker_detected = True
                self.marker_lost_time = 0.0
                
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                tvec = tvecs[0][0]  
                self.marker_position_cam = tvec
                
                aruco.drawDetectedMarkers(debug, corners, ids)
                cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                                 rvecs[0], tvecs[0], 0.1)
                
                cv2.putText(debug, f"MARKER DETECTED! ID: {ids[0][0]}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(debug, f"Distance: {tvec[2]:.2f}m", (10, 65),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug, f"Offset X:{tvec[0]:.2f}m Y:{tvec[1]:.2f}m", (10, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if self.state == DroneState.SCANNING_360:
                    self.state = DroneState.MARKER_TRACKING
                    self.get_logger().info('MARKER FOUND during scan, Switching to tracking')
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_z.reset()
                    self.smoothed_vel = np.zeros(3)
            else:
                self.marker_detected = False
                cv2.putText(debug, "Searching for marker...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            
            state_colors = {
                DroneState.ARMING: (255, 255, 0),
                DroneState.TAKEOFF: (255, 200, 0),
                DroneState.WAYPOINT_SEARCH: (255, 100, 0),
                DroneState.SCANNING_360: (255, 0, 255),
                DroneState.MARKER_TRACKING: (0, 200, 255),
                DroneState.HOVERING: (0, 255, 0)
            }
            
            cv2.putText(debug, f"State: {self.state.name}", (10, debug.shape[0] - 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_colors.get(self.state, (255, 255, 255)), 2)
            cv2.putText(debug, f"Pos: [{self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}]",
                       (10, debug.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug, f"Vel: [{self.current_velocity[0]:.1f}, {self.current_velocity[1]:.1f}, {self.current_velocity[2]:.1f}]",
                       (10, debug.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if self.state == DroneState.SCANNING_360:
                progress = (self.scan_current_yaw / (2 * math.pi)) * 100
                cv2.putText(debug, f"Scan: {progress:.0f}%", (debug.shape[1] - 150, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            cv2.imshow('ArUco Tracker - Camera View', debug)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {str(e)}')
    
    def position_callback(self, msg):
        self.current_position = np.array([msg.x, msg.y, msg.z])
        self.current_velocity = np.array([msg.vx, msg.vy, msg.vz])
        self.current_yaw = msg.heading
    
    def create_trajectory_to(self, target_pos, duration=3.0):
        
        trajectories = []
        
        for i in range(3):
            traj = QuinticTrajectory(
                s0=self.current_position[i],
                sf=target_pos[i],
                v0=self.current_velocity[i],
                vf=0.0,  
                a0=0.0,  
                af=0.0,  
                duration=duration
            )
            trajectories.append(traj)
        
        return trajectories
    
    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        self.state_timer += dt
        
        use_trajectory = self.state in [
        DroneState.TAKEOFF,
        DroneState.WAYPOINT_SEARCH,
        DroneState.MARKER_TRACKING,
        DroneState.HOVERING
        ]        
        self.publish_offboard_control_mode(use_trajectory=use_trajectory)
        
        if self.state == DroneState.ARMING:
            if not self.offboard_enabled:
                self.engage_offboard_mode()
                self.offboard_enabled = True
                self.get_logger().info('Engaging offboard mode...')
            
            if not self.armed and self.state_timer > 1.0:
                self.arm()
                self.armed = True
                self.get_logger().info(' Arming...')
            
            if self.armed and self.state_timer > 2.0:
                self.state = DroneState.TAKEOFF
                self.state_timer = 0
                self.trajectory = self.create_trajectory_to(self.waypoints[0], duration=4.0)
                self.trajectory_start_time = self.get_clock().now()
                self.get_logger().info('Taking off...')
            
            self.publish_position_setpoint(self.current_position, self.current_yaw)
            
        elif self.state == DroneState.TAKEOFF:
            if self.trajectory is not None:
                t = (self.get_clock().now() - self.trajectory_start_time).nanoseconds / 1e9
                
                pos = np.array([traj.position(t) for traj in self.trajectory])
                vel = np.array([traj.velocity(t) for traj in self.trajectory])
                acc = np.array([traj.acceleration(t) for traj in self.trajectory])
                
                self.publish_trajectory_setpoint(pos, vel, acc, self.current_yaw)
                
                if self.trajectory[0].is_finished(t):
                    self.state = DroneState.WAYPOINT_SEARCH
                    self.current_waypoint_idx = 1
                    self.trajectory = None
                    self.get_logger().info('Starting waypoint search...')
                    
        elif self.state == DroneState.WAYPOINT_SEARCH:
            if self.current_waypoint_idx < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint_idx]
                
                if self.trajectory is None:
                    self.trajectory = self.create_trajectory_to(wp, duration=3.0)
                    self.trajectory_start_time = self.get_clock().now()
                    self.get_logger().info(f'Moving to waypoint {self.current_waypoint_idx}')
                
                t = (self.get_clock().now() - self.trajectory_start_time).nanoseconds / 1e9
                pos = np.array([traj.position(t) for traj in self.trajectory])
                vel = np.array([traj.velocity(t) for traj in self.trajectory])
                acc = np.array([traj.acceleration(t) for traj in self.trajectory])
                
                self.publish_trajectory_setpoint(pos, vel, acc, self.current_yaw)
                
                if self.trajectory[0].is_finished(t):
                    self.state = DroneState.SCANNING_360
                    self.scan_start_yaw = self.current_yaw
                    self.scan_current_yaw = 0.0
                    self.scan_target_position = wp.copy()
                    self.trajectory = None
                    self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}, starting 360° scan')
            else:
                self.current_waypoint_idx = 1
                self.trajectory = None
                
        elif self.state == DroneState.SCANNING_360:
            self.scan_current_yaw += self.scan_yaw_rate * dt
            target_yaw = self.scan_start_yaw + self.scan_current_yaw
            
            self.publish_position_setpoint(self.scan_target_position, target_yaw)
            
            if self.scan_current_yaw >= 2 * math.pi:
                self.get_logger().info('360° scan complete, marker not found. Next waypoint...')
                self.current_waypoint_idx += 1
                self.state = DroneState.WAYPOINT_SEARCH
                self.trajectory = None
                
                if self.current_waypoint_idx >= len(self.waypoints):
                    self.get_logger().info('All waypoints scanned, looping...')
                    self.current_waypoint_idx = 1
                    
        elif self.state == DroneState.MARKER_TRACKING:
            if self.marker_detected and self.marker_position_cam is not None:
                self.marker_lost_time = 0.0
                
                x_cam, y_cam, z_cam = self.marker_position_cam
                
                error_x = x_cam  
                error_y = y_cam  
                error_z = z_cam - self.target_distance  

                if abs(error_z) < 0.4 and abs(error_x) < 0.3 and abs(error_y) < 0.3:
                    self.get_logger().info("LOCKED — holding position")
                    self.publish_velocity_setpoint(np.zeros(3))
                    return
                
                
                vel_x_cam = -self.pid_x.compute(error_x, dt)
                vel_y_cam = -self.pid_y.compute(error_y, dt)
                vel_z_cam = -self.pid_z.compute(error_z, dt)
                
                vel_target_ned = np.array([
                    -vel_z_cam,   
                    -vel_x_cam,   
                    -vel_y_cam    
                ])
                
                
                vel_target_ned = np.clip(vel_target_ned, -0.8, 0.8)
                vel_target_ned[0] = max(vel_target_ned[0], -0.3)
                
                error_mag = np.linalg.norm([error_x, error_y, error_z])
                if error_mag < 1:
                    vel_target_ned *= 0.4   
                
                alpha = 0.2
                self.smoothed_vel = alpha * vel_target_ned + (1 - alpha) * self.smoothed_vel
                
                if np.linalg.norm(self.smoothed_vel) > 1.0:
                    self.smoothed_vel = self.smoothed_vel / np.linalg.norm(self.smoothed_vel)
                
                self.publish_velocity_setpoint(self.smoothed_vel)
                
                total_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
                
                self.get_logger().info(
                    f'Tracking - Dist:{z_cam:.2f}m Err:({error_x:.2f},{error_y:.2f},{error_z:.2f}) '
                    f'Vel:[{vel_target_ned[0]:.2f},{vel_target_ned[1]:.2f},{vel_target_ned[2]:.2f}]',
                    throttle_duration_sec=0.5
                )
                
                if total_error < 0.2 and np.linalg.norm(self.smoothed_vel) < 0.25:
                    self.state = DroneState.HOVERING
                    self.get_logger().info(f'HOVERING achieved! Distance: {z_cam:.2f}m')
            else:
                self.marker_lost_time += dt
                
                self.publish_velocity_setpoint(np.zeros(3))
                
                if self.marker_lost_time > self.marker_lost_threshold:
                    self.get_logger().warn('Lost marker, starting 360° scan')
                    self.state = DroneState.SCANNING_360
                    self.scan_start_yaw = self.current_yaw
                    self.scan_current_yaw = 0.0
                    self.scan_target_position = self.current_position.copy()
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_z.reset()
                    
        elif self.state == DroneState.HOVERING:
            if self.marker_detected and self.marker_position_cam is not None:
                self.marker_lost_time = 0.0
                
                x_cam, y_cam, z_cam = self.marker_position_cam
                error_x = x_cam
                error_y = y_cam
                error_z = z_cam - self.target_distance
                
                vel_x_cam = -self.pid_x.compute(error_x, dt)
                vel_y_cam = -self.pid_y.compute(error_y, dt)
                vel_z_cam = -self.pid_z.compute(error_z, dt)
                
                vel_target_ned = np.array([vel_z_cam, vel_x_cam, vel_y_cam])
                alpha = 0.2
                self.smoothed_vel = alpha * vel_target_ned + (1 - alpha) * self.smoothed_vel

                if np.linalg.norm(self.smoothed_vel) > 1.0:
                    self.smoothed_vel = self.smoothed_vel / np.linalg.norm(self.smoothed_vel)

                self.publish_velocity_setpoint(self.smoothed_vel)
                
                
                self.get_logger().info(
                    f'Hovering - Dist:{z_cam:.2f}m Offset:({error_x:.2f},{error_y:.2f})',
                    throttle_duration_sec=2.0
                )
            else:
                self.marker_lost_time += dt
                
                if self.marker_lost_time > self.marker_lost_threshold:
                    self.state = DroneState.MARKER_TRACKING
                    self.get_logger().warn('Lost marker while hovering')
    
    def publish_offboard_control_mode(self, use_trajectory=False):
        msg = OffboardControlMode()
        if use_trajectory:
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
        else:
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_position_setpoint(self, position, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(position[0]), float(position[1]), float(position[2])]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_velocity_setpoint(self, velocity):
        msg = TrajectorySetpoint()
    
        msg.velocity = [float(velocity[0]), float(velocity[1]), float(velocity[2])]
    
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
    
        msg.yaw = float('nan')
    
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def publish_trajectory_setpoint(self, position, velocity, acceleration, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(position[0]), float(position[1]), float(position[2])]
        msg.velocity = [float(velocity[0]), float(velocity[1]), float(velocity[2])]
        msg.acceleration = [float(acceleration[0]), float(acceleration[1]), float(acceleration[2])]
        msg.yaw = float(yaw) if not math.isnan(yaw) else float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    
    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    print('=' * 70)
    print('ArUco Marker Tracker with Quintic Trajectory Control')
    print('=' * 70)
    print('Features:')
    print('  • Smooth quintic trajectories for waypoint navigation')
    print('  • 360° scanning at each waypoint')
    print('  • PID-based marker tracking with trajectory control')
    print('  • Continuous position, velocity, and acceleration control')
    print('=' * 70)
    print('Expected marker: (5, 0, 2.5) world frame')
    print('Target: Hover 2m in front of marker')
    print('=' * 70)
    print('IMPORTANT: Run the Gazebo bridge first:')
    print('ros2 run ros_gz_bridge parameter_bridge \\')
    print('  /realsense/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image')
    print('=' * 70)
    print('Press Ctrl+C to stop')
    print('=' * 70)
    
    rclpy.init(args=args)
    node = ArucoMarkerTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()