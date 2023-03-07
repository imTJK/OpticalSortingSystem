import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from utilities import Stepper

#Goal: Get Classificiations and move the motors accordingly
class ClassificationSubscriber(Node):
    def __init__(self):
        super().__init__('classification_subscriber')
        self.classification_subscriber = self.create_subscription(
            String,
            'classifications',
            self.classification_callback,
            10
        )
        
        self.platform_stepper = Stepper(14, 15, 18, 23)
        self.pusher_stepper = Stepper(24, 25, 8, 7) # Cant use homing function with this one

        ## Stepper Calibration
        #TODO: add switch to detect home position
        self.platform_stepper.home()
        self.pusher_stepper.home()
    
    def classification_callback(self, msg:String):
        match msg.data:
            case 'Plastic':
                pass
            case 'Paper':
                pass
            case 'Glass':
                pass
            case 'Biological':
                pass


def main(args=None):
    rclpy.init(args=args)

    classification_subscriber = ClassificationSubscriber()

    rclpy.spin(classification_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    classification_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
