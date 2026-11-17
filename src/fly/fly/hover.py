#!/usr/bin/env python3
"""
Simple script to manually fly drone to a position where it can see the marker.
This helps with testing and calibration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import cv2.aruco as aruco


class ManualHoverNode(Node):
    """Fly to a specific position and hover there."""
    
    def __init__(self):
        super().__init__('manual_hover')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribers
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.position_callback, qos_profile)
        
        # Camera for visualization
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.camera_sub = self.create_subscription(
            Image, '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.camera_callback, camera_qos)
        
        # ArUco detection for visualization
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        
        # State
        self.position = np.array([0.0, 0.0, 0.0])
        self.armed = False
        self.offboard_enabled = False
        
        # Target position - hover above marker with some offset
        # Marker is at (3, 3, 0), so hover at (3, 3, -3) to see it
        self.target_position = np.array([3.0, 3.0, -3.0])
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Manual Hover Node Started')
        self.get_logger().info(f'Target position: {self.target_position}')
        self.get_logger().info('Press Ctrl+C to stop')
        
    def position_callback(self, msg):
        self.position = np.array([msg.x, msg.y, msg.z])
        
    def camera_callback(self, msg):
        """Show camera feed with ArUco detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            debug = cv_image.copy()
            
            # Draw detection
            if ids is not None:
                aruco.drawDetectedMarkers(debug, corners, ids)
                cv2.putText(debug, f"MARKER DETECTED! ID: {ids[0][0]}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(debug, "No marker visible", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Show current position
            error = np.linalg.norm(self.position - self.target_position)
            cv2.putText(debug, f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug, f"Target: [{self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f}]",
                       (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug, f"Error: {error:.2f}m",
                       (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            cv2.imshow('Camera View', debug)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')
    
    def control_loop(self):
        """Main control loop."""
        # Publish offboard mode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)
        
        # Publish position setpoint
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.position = [float(self.target_position[0]), 
                                 float(self.target_position[1]), 
                                 float(self.target_position[2])]
        setpoint_msg.yaw = 0.0
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(setpoint_msg)
        
        # Enable offboard mode (once)
        if not self.offboard_enabled:
            self.engage_offboard_mode()
            self.offboard_enabled = True
        
        # Arm (once)
        if not self.armed:
            self.arm()
            self.armed = True
        
        # Log progress
        error = np.linalg.norm(self.position - self.target_position)
        if error < 0.5:
            self.get_logger().info(
                f'Near target! Position: {self.position}, Error: {error:.2f}m',
                throttle_duration_sec=2.0
            )
    
    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')
    
    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
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


def main():
    print('Starting Manual Hover to (3, 3, -3)...')
    print('This position should allow the camera to see the marker at (3, 3, 0)')
    rclpy.init()
    node = ManualHoverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
