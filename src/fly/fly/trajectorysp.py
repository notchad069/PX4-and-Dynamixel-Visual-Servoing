import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint

class position(Node):
    def __init__(self):
        super().__init__('trajectorysp')
        self.pub = self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',10)
        self.timer = self.create_timer(0.5,self.timer_callback)

    def timer_callback(self):
        msg = TrajectorySetpoint()
        msg.position= [10.0,10.0,-10.0]
        msg.yaw = 0.0

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub.publish(msg)
        self.get_logger().info('position command given')
    
def main(args=None):
    rclpy.init(args=args)
    node = position()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
