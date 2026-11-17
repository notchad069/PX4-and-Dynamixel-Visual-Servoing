#!/usr/bin/env python3
"""
Diagnostic script to verify camera bridge and ArUco detection.
Run this BEFORE running the main tracker to verify everything works.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class CameraDiagnostics(Node):
    def __init__(self):
        super().__init__('camera_diagnostics')
        
        # Camera QoS
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        
        self.frame_count = 0
        self.marker_detections = 0
        
        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/realsense/color/image_raw',
            self.camera_callback,
            camera_qos
        )
        
        # Status timer
        self.timer = self.create_timer(2.0, self.status_report)
        
        print('=' * 70)
        print('📷 Camera & ArUco Detection Diagnostics')
        print('=' * 70)
        print('Subscribing to: /realsense/color/image_raw')
        print('Waiting for images...')
        print('=' * 70)
        
    def camera_callback(self, msg):
        try:
            self.frame_count += 1
            
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            # Create debug display
            debug = cv_image.copy()
            
            # Add info overlay
            h, w = debug.shape[:2]
            cv2.rectangle(debug, (0, 0), (w, 120), (0, 0, 0), -1)
            
            if ids is not None and len(ids) > 0:
                self.marker_detections += 1
                
                # Draw detections
                aruco.drawDetectedMarkers(debug, corners, ids)
                
                # Estimate pose (approximate camera matrix)
                camera_matrix = np.array([
                    [386.0, 0.0, 320.0],
                    [0.0, 386.0, 240.0],
                    [0.0, 0.0, 1.0]
                ])
                dist_coeffs = np.zeros(5)
                marker_size = 0.2
                
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, marker_size, camera_matrix, dist_coeffs
                )
                
                # Draw axes
                for i in range(len(ids)):
                    cv2.drawFrameAxes(debug, camera_matrix, dist_coeffs, 
                                     rvecs[i], tvecs[i], 0.1)
                
                tvec = tvecs[0][0]
                
                # Display detection info
                cv2.putText(debug, f"✓ MARKER DETECTED!", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(debug, f"ID: {ids[0][0]}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug, f"Dist: {tvec[2]:.2f}m  X:{tvec[0]:.2f}m  Y:{tvec[1]:.2f}m", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(debug, "No markers detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                cv2.putText(debug, "Is marker visible in camera view?", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            # Frame counter
            cv2.putText(debug, f"Frames: {self.frame_count}", (w - 200, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(debug, f"Detections: {self.marker_detections}", (w - 200, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Show
            cv2.imshow('Camera Diagnostics', debug)
            key = cv2.waitKey(1)
            
            if key == ord('q'):
                print('\n👋 User requested quit')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def status_report(self):
        """Print status every 2 seconds."""
        if self.frame_count == 0:
            self.get_logger().warn('⚠️  No frames received yet!')
            self.get_logger().warn('Check if bridge is running:')
            self.get_logger().warn('  ros2 run ros_gz_bridge parameter_bridge /realsense/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image')
        else:
            detection_rate = (self.marker_detections / self.frame_count) * 100
            self.get_logger().info(
                f'📊 Frames: {self.frame_count}, Detections: {self.marker_detections} ({detection_rate:.1f}%)'
            )

def main():
    rclpy.init()
    node = CameraDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n👋 Shutting down diagnostics')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
