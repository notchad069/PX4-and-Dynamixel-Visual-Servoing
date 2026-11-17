import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.listener_callback,
            10)
        self.br = CvBridge()
    
    def listener_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        current_frame_gray = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)
        current_frame_gray_blur = cv2.GaussianBlur(current_frame_gray,(3,3),0)
        current_frame_gray_blur_canny = cv2.Canny(current_frame_gray_blur,threshold1=100,threshold2=200)

        ret,thresh = cv2.threshold(current_frame_gray_blur_canny,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        self.get_logger().info(f'Contours found: {len(contours)}')

        for c in contours:
            M = cv2.moments(c)
            cx = int(M['m10']/M['m00']) if M['m00']!= 0 else 0
            cy = int(M['m01']/M['m00']) if M['m00']!= 0 else 0
            cv2.circle(current_frame_gray_blur_canny,(cx,cy),5,(255,255,255),-1)
            
            cv2.imshow("camera", current_frame_gray_blur_canny)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()