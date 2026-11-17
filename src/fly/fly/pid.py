import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleThrustSetpoint,
    VehicleCommand,
    OffboardControlMode,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class AltitudePIDController(Node):
    def __init__(self):
        super().__init__('pid_node')

        # === PID parameters ===
        self.kp = 0.8
        self.ki = 0.05
        self.kd = 0.1

        # === State variables ===
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_altitude = 0.0
        self.target_altitude = -5.0  # NED frame (Z is negative upward)

        # === PX4 publishers ===
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10
        )
        self.thrust_pub = self.create_publisher(
            VehicleThrustSetpoint,
            "/fmu/in/vehicle_thrust_setpoint",
            10
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10
        )

        # === Subscriber with proper QoS ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.position_callback,
            qos_profile
        )

        # === Timer for control loop (50 Hz) ===
        self.timer = self.create_timer(0.02, self.control_loop)

        # === State machine variables ===
        self.loop_counter = 0
        self.offboard_mode_sent = False
        self.armed = False
        self.position_received = False

        self.get_logger().info("Altitude PID Node initialized")

    # ------------------------------
    # PX4 Callbacks & Publishers
    # ------------------------------
    def position_callback(self, msg):
        self.current_altitude = msg.z
        if not self.position_received:
            self.get_logger().info("Receiving /vehicle_local_position messages")
            self.position_received = True

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Sent ARM command")

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Sent DISARM command")

    def set_offboard_mode(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Sent OFFBOARD mode command")

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True  # PX4 expects this when using thrust setpoints
        self.offboard_pub.publish(msg)

    def publish_thrust(self, thrust_z):
        msg = VehicleThrustSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.xyz = [0.0, 0.0, thrust_z]
        self.thrust_pub.publish(msg)

    # ------------------------------
    # Main Control Loop
    # ------------------------------
    def control_loop(self):
        # Keep publishing offboard heartbeat even before arming
        self.publish_offboard_mode()

        self.loop_counter += 1

        # Step 1: Wait for PX4 connection & stable publishing
        if not self.position_received:
            return

        # Step 2: Send OFFBOARD command after ~1 second
        if self.loop_counter == 50:
            self.set_offboard_mode()

        # Step 3: Arm after another second
        elif self.loop_counter == 100:
            self.arm()

        # Step 4: Run PID control after arming
        elif self.loop_counter > 100:
            error = self.target_altitude - self.current_altitude
            self.integral += error * 0.02
            derivative = (error - self.prev_error) / 0.02

            thrust = self.kp * error + self.ki * self.integral + self.kd * derivative

            # Clamp thrust between 0 and 1
            thrust = max(0.0, min(1.0, thrust))

            self.publish_thrust(thrust)
            self.get_logger().info(
                f"Alt={self.current_altitude:.2f} | Err={error:.2f} | Thrust={thrust:.3f}"
            )

            self.prev_error = error


def main(args=None):
    rclpy.init(args=args)
    node = AltitudePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt → Disarming...")
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

