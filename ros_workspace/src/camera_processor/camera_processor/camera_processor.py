import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

##libraries imported for Advanced 1
import numpy as np


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

        # Added a check for Advanced 1 functionality
        if self.advanced_1_enabled:
            self.advanced_image_processing(image)
        else:
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



    def advanced_image_processing(self, image):
        #the kernel for transforming the image
        kernel = np.ones((5,5),np.float32)/25 # averaging filter kernel

        match self.effect_counter:
            case 0:
                distorted_image = cv2.filter2D(image,-1,kernel) #applying the filter
            case 1:
                distorted_image = cv2.medianBlur(distorted_image,5) #applying median blur to reduce noise
            case 2:
                distorted_image = cv2.bilateralFilter(distorted_image,9,75,75) #applying bilateral filter to preserve edges
            case 3:
                dustyness = 0.02  #percentage of image to be affected by dust
                distorted_image = self.dust_filter(image, dustyness)  #using salt and pepper filter to make dust effect
            case 4: # dustier version of the version above
                dustyness = 0.05  #percentage of image to be affected by dust
                distorted_image = self.dust_filter(image, dustyness)  #using salt and pepper filter to make dust effect
            case 5:
                motion_bluriness = 15
                distorted_image = self.motion_blur(image, motion_bluriness)  #applying motion blur to the image
            case 6:
                motion_bluriness = 30
                distorted_image = self.motion_blur(image, motion_bluriness)  #applying more motion blur to the image

        self.image_saver(image)  #save the original image
        self.image_saver(distorted_image)  #save the processed image instead of the original
        
        self.effect_counter +=1
        if self.effect_counter > 6:
            self.effect_counter = 0  #reset the counter after all effects have been used

    def dust_filter(self, image, dustyness):
        #Function to apply salt and pepper noise to an image to simulate dust
        noisy_image = np.copy(image)
        x, y, ch = noisy_image.shape
        noisy_pixels = int(x * y * dustyness)

        for _ in range(noisy_pixels):
            row, col = np.random.randint(0, x), np.random.randint(0, y)
            noisy_image[row, col] = [88, 57, 39]  # brownish color for dust

        return noisy_image

    def motion_blur(self, image, motion_bluriness):
        #Function to apply motion blur to an image
        kernel = np.zeros((motion_bluriness, motion_bluriness)) #the size of the kernel affects the amount of blur
        kernel[int((motion_bluriness - 1)/2), :] = np.ones(motion_bluriness) #creates  blur that goes horizontally acros the screen
        """
        e.g.
           [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0
            1, 1, 1, 1, 1,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        
            if the 1's went down the matrix vertically it would create a veritical motion blur
        
        """
        kernel = kernel / motion_bluriness  #normalising the kernel
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