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
        model.conf = 0.9

        try: 
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            results = model(source=img, verbose=False)[0].cpu()
            
            classes = results.names
            found = []
            for x in results.boxes.cls:
                found.append(classes.get(int(x.item())))

            self.get_logger().info(str(found))
        except CvBridgeError as e:
            self.get_logger().error(str(e))
        
        #self.classification_publisher.publish()

def main(args=None):
    rclpy.init(args=args)

    classification_subscriber = Classifier()

    rclpy.spin(classification_subscriber)

    classification_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()