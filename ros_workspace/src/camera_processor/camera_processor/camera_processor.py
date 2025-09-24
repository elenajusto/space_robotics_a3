import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# This is a custom node which we are using to listen to the camera
class CameraProcessor(Node):

    def __init__(self):
        # Node initialisation
        super().__init__('camera_processor')

         # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Subscribes to the camera sensor's topic which has messages which contain image
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)

        # Debug statement
        print("Initialised camera_processor")       

    # Method within the custom node that is called when data (image messages) from camera topic is received
    def image_callback(self, msg):
        
        # Debug statement
        print("Doing the listen...")                

        # Convert received message into cv2 format
        image = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Debug
        print(image)  # Results: Images are outputted in matrix form

    def image_saver(self, image):
        # TODO: Handle the process of saving an image to local file
        
        pass # TODO: Remove when implemented

def main(args=None):
    rclpy.init(args=args)

    print("Initialising camera_processor...")     
    
    camera_processor = CameraProcessor()

    rclpy.spin(camera_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()