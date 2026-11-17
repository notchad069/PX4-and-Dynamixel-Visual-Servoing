#!/usr/bin/env python3

import sys
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

moveBindings = {
    'w': (0.0, 0.0, -0.5),  # Z down (PX4 NED frame)
    's': (0.0, 0.0, 0.5),   # Z up
    'a': (0.0, -0.5, 0.0),  # Y left
    'd': (0.0, 0.5, 0.0),   # Y right
    '\x1b[A': (0.5, 0.0, 0.0),   # X forward
    '\x1b[B': (-0.5, 0.0, 0.0),  # X backward
    '\x1b[C': (0.0, 0.5, 0.0),   # Y right
    '\x1b[D': (0.0, -0.5, 0.0),  # Y left
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardOffboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_offboard_control')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.x = 0.0
        self.y = 0.0
        self.z = -1.5  # Hover height
        self.armed = False
        self.offboard_mode_engaged = False
        self.start_time = time.time()
        self.setpoints_sent = 0

    def timer_callback(self):
        self.publish_offboard_control_heartbeat()
        self.publish_position_setpoint()

    def publish_offboard_control_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.x, self.y, self.z]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)
        self.setpoints_sent += 1

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Sent arm command")

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Sent disarm command")

    def engage_offboard(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Sent offboard mode command")


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = KeyboardOffboardControl()

    # Spin the ROS node in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("""
Control the drone with your keyboard:
W/S: up/down
A/D: left/right
Arrow keys: forward/backward
Space: arm/disarm (after 10 setpoints)
Ctrl+C to quit
""")

    try:
        while True:
            key = getKey()
            if key in moveBindings:
                dx, dy, dz = moveBindings[key]
                node.x += dx
                node.y += dy
                node.z += dz
                print(f"Setpoint -> X:{node.x:.1f}, Y:{node.y:.1f}, Z:{node.z:.1f}")

            elif key == ' ':
                if node.setpoints_sent < 10:
                    print("❌ Wait for at least 10 setpoints before arming.")
                    continue

                if not node.offboard_mode_engaged:
                    node.engage_offboard()
                    node.offboard_mode_engaged = True
                    continue

                node.armed = not node.armed
                if node.armed:
                    node.arm()
                else:
                    node.disarm()

            elif key == '\x03':  # Ctrl+C
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
