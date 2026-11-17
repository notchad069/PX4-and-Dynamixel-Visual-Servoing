import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class vehiclecommand(Node):
    def __init__(self):
        super().__init__('vehiclecmd')
        self.pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
        self.create_timer(0.5,self.timer_callback)
        self.sent = False
    
    def timer_callback(self):
        if self.sent:
            return
        
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_cmd = VehicleCommand()
        offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_cmd.param1 = 1.0
        offboard_cmd.param2 = 6.0
        offboard_cmd.target_system = 1
        offboard_cmd.target_component = 1
        offboard_cmd.from_external = True
        offboard_cmd.timestamp = timestamp
        self.pub.publish(offboard_cmd)
        self.get_logger().info('offboard cmds sent chief')

        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0
        arm_cmd.target_system = 1
        arm_cmd.target_component = 1
        arm_cmd.from_external = True
        arm_cmd.timestamp = timestamp
        self.pub.publish(arm_cmd)
        self.get_logger().info('armed')

        self.sent = True

def main(args = None):
    rclpy.init(args = args)
    node = vehiclecommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == "__main__":
    main()
