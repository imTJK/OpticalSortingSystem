import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError
       
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        timer_period = 1/5

        self.publisher = self.create_publisher(Image, 'images', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            ret, frame = self.cap.read() 
            if ret:
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        except CvBridgeError as e:
            self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()