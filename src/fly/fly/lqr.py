#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np
from scipy.linalg import solve_continuous_are


class LQRHoverController(Node):
    """Node for LQR-based hover control at a single setpoint."""

    def __init__(self) -> None:
        super().__init__('lqr_hover_controller')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        # Hover setpoint (x, y, z)
        self.setpoint = np.array([0.0, 0.0, -5.0])
        
        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        
        # LQR gains will be computed
        self.K = None
        self.compute_lqr_gain()
        
        # Control limits
        self.max_velocity = 2.0  # m/s
        
        # Flags
        self.armed = False
        self.offboard_enabled = False
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'LQR Hover Controller initialized. Target: {self.setpoint}')

    def compute_lqr_gain(self):
        """Compute LQR gain matrix for hover control."""
        # State-space model: dx/dt = Ax + Bu
        # State: [x, y, z, vx, vy, vz]
        # Control: [ax, ay, az] (acceleration commands)
        
        # System matrix A (simple double integrator model)
        A = np.array([
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ])
        
        # Input matrix B
        B = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        
        # Cost matrices
        # Q penalizes state error (position and velocity)
        Q = np.diag([10.0, 10.0, 15.0, 1.0, 1.0, 2.0])  # Higher weight on z-axis
        
        # R penalizes control effort
        R = np.diag([1.0, 1.0, 1.0])
        
        # Solve continuous-time algebraic Riccati equation
        try:
            P = solve_continuous_are(A, B, Q, R)
            self.K = np.linalg.inv(R) @ B.T @ P
            self.get_logger().info('LQR gain matrix computed successfully')
            self.get_logger().info(f'Gain matrix K:\n{self.K}')
        except Exception as e:
            self.get_logger().error(f'Failed to compute LQR gain: {e}')
            # Fallback to simple PD gains if LQR computation fails
            self.K = np.array([
                [3.0, 0.0, 0.0, 1.5, 0.0, 0.0],
                [0.0, 3.0, 0.0, 0.0, 1.5, 0.0],
                [0.0, 0.0, 4.0, 0.0, 0.0, 2.0]
            ])
            self.get_logger().warn('Using fallback PD gains')

    def vehicle_local_position_callback(self, msg):
        """Update current state from vehicle position."""
        self.vehicle_local_position = msg
        self.state = np.array([
            msg.x, msg.y, msg.z,
            msg.vx, msg.vy, msg.vz
        ])

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def compute_control(self):
        """Compute LQR control based on current state."""
        # State error
        state_error = self.state.copy()
        state_error[0:3] -= self.setpoint
        
        # LQR control law: u = -K * x_error
        # This gives us desired accelerations, but we'll convert to velocities
        control_acceleration = -self.K @ state_error
        
        # Convert acceleration to velocity command (simple integration)
        dt = 0.1
        desired_velocity = self.state[3:6] + control_acceleration * dt
        
        # Limit velocity
        vel_norm = np.linalg.norm(desired_velocity)
        if vel_norm > self.max_velocity:
            desired_velocity = desired_velocity / vel_norm * self.max_velocity
        
        return desired_velocity

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        """Publish trajectory setpoint with LQR control."""
        msg = TrajectorySetpoint()
        
        # Compute LQR control
        desired_velocity = self.compute_control()
        
        # Set position and velocity
        msg.position = [float(self.setpoint[0]), float(self.setpoint[1]), float(self.setpoint[2])]
        msg.velocity = [float(desired_velocity[0]), float(desired_velocity[1]), float(desired_velocity[2])]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.trajectory_setpoint_publisher.publish(msg)
        
        # Log position error
        pos_error = np.linalg.norm(self.state[0:3] - self.setpoint)
        if pos_error < 0.1:
            self.get_logger().info(f'Hovering at setpoint (error: {pos_error:.3f}m)', throttle_duration_sec=5.0)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
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
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Publish offboard control mode
        self.publish_offboard_control_heartbeat_signal()
        
        # Publish position setpoint with LQR control
        self.publish_position_setpoint()
        
        # Enable offboard mode and arm
        if not self.offboard_enabled:
            self.engage_offboard_mode()
            self.offboard_enabled = True
            
        if not self.armed:
            self.arm()
            self.armed = True


def main(args=None) -> None:
    print('Starting LQR hover controller node...')
    rclpy.init(args=args)
    lqr_controller = LQRHoverController()
    rclpy.spin(lqr_controller)
    lqr_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)