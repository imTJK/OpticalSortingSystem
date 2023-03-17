#ROS
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

#Computer-Vision
import cv2
from cv_bridge import CvBridge, CvBridgeError

#Local   
from .stepper import Stepper

import time

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        #Setup for circular buffer, initalized with "Empty"-Values
        self.classification_buffer_size = 10
        self.prev_classifications = [''] * self.classification_buffer_size

        #Setup for Stepper-Motors
        self.platform_stepper = Stepper(14, 15, 18, 23)
        self.pusher_stepper = Stepper(24, 25, 8, 7) # Cant use homing function with this one
        
        #Computer-Vision Setup, continuous image capturing from camera-device 0 (RasPi-Cam)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        time.sleep(2)

        self.publisher = self.create_publisher(Image, 'images', 10)
        self.classification_subscription = self.create_subscription(String, "classifications", self.motor_control, 10)

        self.timer_period = 1/5
        self.publisher_timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        #Captures Images, converts them to ROS-Messages with cv2Bridge and publishes them in the "images" topic

        try:
            ret, frame = self.cap.read() 
            if ret:
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        except CvBridgeError as e:
            self.get_logger().error(e)

    def motor_control(self, msg:String):
        #Listens for incoming classifications of previously published images and steers the motors accordingly
        # - 

        if all(ele == msg.data and ele != '' for ele in self.prev_classifications):
            #Stops sending images while the object is being moved/sorted and starts again as soon as it's done
            self.get_logger().info(f"Detected {msg.data}")
            self.publisher_timer.destroy()
            match msg.data:
                case 'plastic':
                    pass
                case 'paper':
                    pass
                case 'cardboard':
                    pass
                case 'glass':
                    pass
            self.prev_classifications = [''] * self.classification_buffer_size
            self.publisher_timer = self.create_timer(self.timer_period, self.timer_callback)

        self.prev_classifications.pop(0)
        self.prev_classifications.append(msg.data)

def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()