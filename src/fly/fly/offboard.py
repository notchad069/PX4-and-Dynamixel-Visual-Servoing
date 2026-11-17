import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode

class offboardcontrol(Node):
    def __init__(self):
        super().__init__('offboard')
        self.offb = self.create_publisher(OffboardControlMode,'fmu/in/offboard_control_mode',10)
        self.timer = self.create_timer(0.5,self.timer_callback)
    
    def timer_callback(self):
        msg = OffboardControlMode()
        msg.position = True 
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.offb.publish(msg)
        self.get_logger().info('offboard controlmode publishin...')

def main(args = None):
        rclpy.init(args = args)
        node = offboardcontrol()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
