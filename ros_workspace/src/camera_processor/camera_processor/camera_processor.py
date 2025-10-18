import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

##libraries imported for Advanced 1
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from PIL import Image, ImageEnhance


# Configure
local_image_directory = r"raw_images"

# This is a custom node which we are using to listen to the camera
class CameraProcessor(Node):

    def __init__(self):
        # Node initialisation
        super().__init__('camera_processor')

         # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        self.advanced_1_enabled = False  # Set to True to enable Advanced 1 functionality
        #currently needs to be changed manually
        self.effect_counter = 0  # Counter to cycle through different effects in Advanced 1
        self.image_before_pub_ = self.create_publisher(Image, 'camera/image_before', 1)
        self.image_after_pub_ = self.create_publisher(Image, 'camera/image_after', 1)
        self.image_annot_pub_ = self.create_publisher(Image, 'camera/image_detections', 1)

        # Parameters to switch effects at runtime (ros2 param set …)
        self.declare_parameter('effect', 'none')     # 'none' | 'dust' | 'motion_blur' | 'low_light' | 'low_res'
        self.declare_parameter('severity', 0)        # 0..3
        self.declare_parameter('chain_preprocess', True)  # run improvement before YOLO
        self.declare_parameter('save_every_n', 0)    # 0 = don’t save, else save every N frames


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

        image = self.advanced_1_image_processing(image)
        
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


    def advanced_1_image_processing(self, image):
        #the kernel for transforming the image
        effect = self.get_parameter('effect').get_parameter_value().string_value
        level  = int(self.get_parameter('severity').get_parameter_value().integer_value)

        """
        effect options:
        "none" : no effect
        "blur" : median blur filter
        "dust" : salt and pepper noise to simulate dust
        "motion_blur" : motion blur filter
        "low_light" : simulating low light conditions
        "resolution" : reducing image resolution
        """

        match effect:
            case "none": # no effect
                return image  #no processing done
            case "blur": # median blur filter
                distorted_image = cv2.medianBlur(distorted_image,5) #applying median blur to reduce noise
            case "dust": # adding dust to the image
                distorted_image = self.dust_filter(image, level)  #using salt and pepper filter to make dust effect
            case "motion_blur": # applying motion blur to the image
                distorted_image = self.motion_blur(image, level)
            case "low_light": # simulating low light conditions
                distorted_image = self.low_light(image, level) 
            case "resolution": # reducing image resolution
                distorted_image

            ###could apply functions that combine two effects


        self.image_saver(image)  #save the original image
        self.image_saver(distorted_image)  #save the processed image instead of the original

    def resolution_reduction(self, image, reduction_level):
        if reduction_level == 0:
            self._logger.warn("Resolution reduction level 0 selected, no processing will be done.")
            return image  #no processing done
        scale = {1: 0.75, 2: 0.5, 3: 0.25}.get(reduction_level, 0.5)  #different levels of resolution reduction

        # Get original dimensions
        height, width = image.shape[:2]

        # Calculate new dimensions
        new_width = int(width * scale)
        new_height = int(height * scale)

        # Resize down
        low_res_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

        # Resize back up to original size
        restored_image = cv2.resize(low_res_image, (width, height), interpolation=cv2.INTER_NEAREST)

        return restored_image

    def low_light(self, image, dimness):
        if dimness == 0:
            self._logger.warn("Low light level 0 selected, no processing will be done.")
            return image  #no processing done
        darkness = {1: -20, 2: -40, 3: -60}.get(dimness, 0.5)
        dark_image = cv2.addWeighted(image, 1.0, np.zeros_like(image), 0.0, darkness)

        return dark_image

    def dust_filter(self, image, dustiness):
        #Function to apply salt and pepper noise to an image to simulate dust
        if dustiness == 0:
            self._logger.warn("Dust level 0 selected, no processing will be done.")
            return image  #no processing done
        dust = {1:0.01, 2:0.03, 3:0.05}.get(dustiness, 0.01)  #different levels of dustiness

        x, y, ch = image.shape
        noisy_pixels = int(x * y * dust)

        for _ in range(noisy_pixels):
            row, col = np.random.randint(0, x), np.random.randint(0, y)
            image[row, col] = [88, 57, 39]  # brownish color for dust

        return image

    def motion_blur(self, image, bluriness):
        if bluriness == 0:
            self._logger.warn("Motion blur level 0 selected, no processing will be done.")
            return image  #no processing done
        blur = {1: 10, 2: 20, 3: 30}.get(bluriness, 10)
        #Function to apply motion blur to an image
        kernel = np.zeros((blur, blur)) #the size of the kernel affects the amount of blur
        kernel[int((blur - 1)/2), :] = np.ones(blur) #creates  blur that goes horizontally acros the screen
        """
        e.g.
           [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0
            1, 1, 1, 1, 1,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        
            if the 1's went down the matrix vertically it would create a veritical motion blur
        
        """
        kernel = kernel / bluriness  #normalising the kernel
        blurred_image = cv2.filter2D(image, -1, kernel)
        return blurred_image


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