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
        
        self.classification_buffer_size = 100
        self.classification_entry = 0
        self.prev_classifications = [None] * self.classification_buffer_size

        self.platform_stepper = Stepper(14, 15, 18, 23)
        self.pusher_stepper = Stepper(24, 25, 8, 7) # Cant use homing function with this one

        ## Stepper Calibration
        #TODO: add switch to detect home position
        self.platform_stepper.home()
        self.pusher_stepper.home()
    
    def classification_callback(self, msg:String):
        if all(ele == msg.data for ele in self.prev_classifications):
            match msg.data:
                case 'Plastic':
                    pass
                case 'Paper':
                    pass
                case 'Glass':
                    pass
                case 'Biological':
                    pass
            self.prev_classifications = []

        self.prev_classifications[self.classification_entry % self.classification_buffer_size] = msg.data 


def main(args=None):
    rclpy.init(args=args)
    classification_subscriber = ClassificationSubscriber()
    rclpy.spin(classification_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
