import rclpy
import os
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError


#Goal: Get Classificiations and move the motors accordingly
class Classifier(Node):
    def __init__(self):
        super().__init__('classifier')
        
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(Image, 'images', self.image_classification, 10)
        self.classification_publisher = self.create_publisher(String, 'classifications', 10)

    def image_classification(self, msg:Image):
        model = YOLO(f" {os.getcwd()}/src/optical_sorting_system/optical_sorting_system/pc/computer_vision/weights/best.pt")  # load the pretrained model
        model.conf = 0.8
        
        pub_msg = String()

        try: 
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            results = model(source=img, verbose=False)[0].cpu()
            
            classes = results.names
            cls = [x.item() for x in results.boxes.cls]
            conf = [x.item() for x in results.boxes.conf]
            max_conf = max(conf) if conf else 0
            
            if max_conf >= model.conf:
                # Possible, but highly unlikely chance of equal equal confidence values
                pub_msg.data = classes.get(cls[conf.index(max_conf)]) 
            else: pub_msg.data = ''
             
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        
        self.get_logger().info(f"Detected {pub_msg.data}")
        self.classification_publisher.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        classification_subscriber = Classifier()
        rclpy.spin(classification_subscriber)
    except KeyboardInterrupt:
        pass

    classification_subscriber.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()