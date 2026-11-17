#!/usr/bin/env python3
"""
Diagnostic tool to figure out the correct marker size and camera parameters.
Run this while the drone is hovering to see what ArUco detects.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import cv2.aruco as aruco


class ArUcoDiagnostic(Node):
    def __init__(self):
        super().__init__('aruco_diagnostic')
        
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        
        # Try different marker sizes
        self.marker_sizes_to_test = [0.15, 0.5, 1.0, 1.5]
        self.current_marker_size = 0.5
        
        # Camera intrinsics
        self.camera_matrix = np.array([
            [277.191356, 0.0, 160.0],
            [0.0, 277.191356, 120.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)
        
        # Drone position
        self.drone_position = np.array([0.0, 0.0, 0.0])
        
        # Subscribers
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.camera_callback,
            qos_sensor
        )
        
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_px4
        )
        
        self.get_logger().info('ArUco Diagnostic Tool Started')
        self.get_logger().info('Press keys in the OpenCV window:')
        self.get_logger().info('  1/2/3/4: Test marker sizes 0.15m / 0.5m / 1.0m / 1.5m')
        self.get_logger().info('  q: Quit')
        
    def position_callback(self, msg):
        self.drone_position = np.array([msg.x, msg.y, msg.z])
        
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            # Create debug image
            debug = cv_image.copy()
            
            if ids is not None and len(ids) > 0:
                # Draw detected markers
                aruco.drawDetectedMarkers(debug, corners, ids)
                
                # Test all marker sizes
                for i, test_size in enumerate(self.marker_sizes_to_test):
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners, test_size, self.camera_matrix, self.dist_coeffs
                    )
                    
                    tvec = tvecs[0][0]
                    distance = np.linalg.norm(tvec)
                    
                    # Calculate what the world position would be (simplified)
                    # Assuming camera points forward/down
                    estimated_world = self.drone_position + np.array([tvec[2], tvec[0], tvec[1]])
                    error_from_target = np.linalg.norm(estimated_world - np.array([3.0, 3.0, 0.0]))
                    
                    color = (0, 255, 0) if test_size == self.current_marker_size else (150, 150, 150)
                    y_pos = 30 + i * 25
                    
                    cv2.putText(
                        debug,
                        f"Size {test_size}m: dist={distance:.2f}m, world=[{estimated_world[0]:.1f},{estimated_world[1]:.1f},{estimated_world[2]:.1f}], err={error_from_target:.2f}m",
                        (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        color,
                        1
                    )
                
                # Draw axes for current marker size
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.current_marker_size, self.camera_matrix, self.dist_coeffs
                )
                cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                                 rvecs[0], tvecs[0], self.current_marker_size * 0.5)
                
                # Show current state
                cv2.putText(debug, f"Drone: [{self.drone_position[0]:.2f}, {self.drone_position[1]:.2f}, {self.drone_position[2]:.2f}]",
                           (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(debug, f"Target marker at: [3.0, 3.0, 0.0]",
                           (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(debug, f"Current marker size: {self.current_marker_size}m (press 1-4 to change)",
                           (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
            else:
                cv2.putText(debug, "No markers detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.putText(debug, f"Drone at: [{self.drone_position[0]:.2f}, {self.drone_position[1]:.2f}, {self.drone_position[2]:.2f}]",
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow('ArUco Diagnostic', debug)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('1'):
                self.current_marker_size = 0.15
                self.get_logger().info(f'Marker size set to {self.current_marker_size}m')
            elif key == ord('2'):
                self.current_marker_size = 0.5
                self.get_logger().info(f'Marker size set to {self.current_marker_size}m')
            elif key == ord('3'):
                self.current_marker_size = 1.0
                self.get_logger().info(f'Marker size set to {self.current_marker_size}m')
            elif key == ord('4'):
                self.current_marker_size = 1.5
                self.get_logger().info(f'Marker size set to {self.current_marker_size}m')
            elif key == ord('q'):
                self.get_logger().info('Quitting...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main():
    rclpy.init()
    node = ArUcoDiagnostic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
