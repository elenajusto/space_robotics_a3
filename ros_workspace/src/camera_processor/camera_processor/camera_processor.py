import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        print("Initialised camera_processor")       # TODO: Debug

    def listener_callback(self, msg):
        print("Doing the listen...")                # TODO: Debug
        print("Subscription: ", self.subscription)  # TODO: Debug
        print("Message: ", msg)                     # TODO: Debug


def main(args=None):
    rclpy.init(args=args)

    print("Initialising camera_processor...")       # TODO: Debug
    
    camera_processor = CameraProcessor()

    rclpy.spin(camera_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()