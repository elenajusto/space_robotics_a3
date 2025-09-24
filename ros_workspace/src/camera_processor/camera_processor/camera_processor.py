import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

# Configure
local_image_directory = r"raw_images"

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

    def image_callback(self, msg):
        '''
        Method within the custom node that is called when data (image messages) from camera topic is received
        '''
        
        # Debug statement
        print("Doing the listen...")                

        # Convert received message into cv2 format
        image = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Debug
        print(image)  # Results: Images are outputted in matrix form

        # Call image_saver
        self.image_saver(image)

    def image_saver(self, image):
        '''
        Handle the process of saving an image to local file
        '''

        # Create image directory if needed 
        os.makedirs(local_image_directory, exist_ok=True) 

        # Create filename with timestamp
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"camera_image_{timestamp}.jpg"
        filepath = os.path.join(local_image_directory, filename)

        # Make image RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Save image to directory using cv2
        cv2.imwrite(filepath, image)
        print(f"Saved image to {filepath}")

        time.sleep(3)  # Pause execution for 3 seconds

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