import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np
from rclpy.qos import qos_profile_sensor_data


local_image_directory = "advanced_1_raw_and_effects"

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.cv_bridge_ = CvBridge()

        # Publishers: before/after streams for RViz
        self.pub_before = self.create_publisher(Image, 'camera/image_before', 1)
        self.pub_after  = self.create_publisher(Image, 'camera/image_after',  1)

        # Runtime parameters (set with ros2 param set …)
        self.declare_parameter('effect', 'none')      # none | dust | motion_blur | low_light | low_res | blur
        self.declare_parameter('severity', 0)         # 0..3
        self.declare_parameter('save_every_n', 0)     # 0 = no save, else save every N frames
        self.frame_idx = 0

        # Sub (sensor QoS recommended)
        self.sub = self.create_subscription(
            Image, 'camera/image', self.image_callback, qos_profile_sensor_data
        )

        # CSV log for your report (what effect/level was active when an image was saved)
        os.makedirs(local_image_directory, exist_ok=True)
        self.log_path = os.path.join(local_image_directory, 'advanced1_log.csv')
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w') as f:
                f.write('timestamp,effect,severity,frame_idx,filename\n')

        self.get_logger().info("CameraProcessor (Advanced 1) initialised")

    """def image_callback(self, msg):
        img = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Publish BEFORE stream
        self.pub_before.publish(self.cv_bridge_.cv2_to_imgmsg(img, encoding='rgb8'))

        # Build AFTER (degraded) image
        effect  = self.get_parameter('effect').get_parameter_value().string_value
        level   = int(self.get_parameter('severity').get_parameter_value().integer_value)
        level   = max(0, min(level, 3))

        degraded = self.apply_effect(img, effect, level)

        # Publish AFTER stream
        self.pub_after.publish(self.cv_bridge_.cv2_to_imgmsg(degraded, encoding='rgb8'))

        # Optional saving cadence
        n = int(self.get_parameter('save_every_n').get_parameter_value().integer_value)
        if n > 0 and (self.frame_idx % n == 0):
            ts = time.strftime("%Y%m%d-%H%M%S")
            fn_before = f"before_{ts}.jpg"
            fn_after  = f"after_{ts}_{effect}_L{level}.jpg"
            p_before = os.path.join(SAVE_DIR, fn_before)
            p_after  = os.path.join(SAVE_DIR, fn_after)
            cv2.imwrite(p_before, cv2.cvtColor(img,      cv2.COLOR_BGR2RGB))
            cv2.imwrite(p_after,  cv2.cvtColor(degraded, cv2.COLOR_BGR2RGB))
            with open(self.log_path, 'a') as f:
                f.write(f'{ts},{effect},{level},{self.frame_idx},{fn_after}\n')
            self.get_logger().info(f"Saved {fn_before}, {fn_after}")

        self.frame_idx += 1"""
    
    def image_callback(self, msg):
        # Method within the custom node that is called when data (image messages) from camera topic is received
        # Convert received message into cv2 format
        image = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Debug
        print(image)  # Results: Images are outputted in matrix form

        self.image_before_pub_.publish(self.cv_bridge_.cv2_to_imgmsg(image, encoding='passthrough'))

        # Debug statement
        print("Doing the listen...")
        n = int(self.get_parameter('save_every_n').get_parameter_value().integer_value) #get the save every n frames parameter
        if n > 0 and self.frame_idx % n == 0: #check if we need to save this frame by checking the frame index
            effected_image = self.advanced_1_image_processing(image) #process the image with selected effect

            self.image_after_pub_.publish(self.cv_bridge_.cv2_to_imgmsg(effected_image, encoding='passthrough')) #publish the effected image

            self.image_saver(effected_image)  #save the effected image
        
        self.frame_idx += 1
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

    # ----- Effects -----
    def apply_effect(self, image, effect, level):
        if level == 0 or effect == 'none':
            return image

        if effect == 'blur':
            # median blur to simulate mild defocus
            k = {1:3, 2:5, 3:7}[level]
            return cv2.medianBlur(image, k)

        if effect == 'dust':
            return self.dust_filter(image, level)

        if effect == 'motion_blur':
            return self.motion_blur(image, level)

        if effect == 'low_light':
            return self.low_light(image, level)

        if effect == 'low_res':
            return self.low_res(image, level)

        self.get_logger().warn(f"Unknown effect '{effect}', passing through")
        return image

    def low_res(self, image, level):
        if level == 0:
            self.get_logger().warn("Resolution reduction level 0 selected, no processing will be done.")
            return image  #no processing done
        scale = {1: 0.75, 2: 0.5, 3: 0.25}.get(level, 0.5)  #different levels of resolution reduction

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

    def low_light(self, image, level):
        # shift brightness negative; combine w/ contrast clip (CLAHE) in compensation step before YOLO
        """shift = {1: -30, 2: -60, 3: -90}[level]
        return cv2.add(image, np.array([shift], dtype=np.int16)).clip(0,255).astype(np.uint8)"""
        if level == 0:
            self.get_logger().warn("Low light level 0 selected, no processing will be done.")
            return image  #no processing done
        darkness = {1: -30, 2: -60, 3: -90}.get(level, 0.5)
        dark_image = cv2.addWeighted(image, 1.0, np.zeros_like(image), 0.0, darkness)

        return dark_image

    def dust_filter(self, image, level):
        # non-destructive: work on a copy & pepper with brown specs
        """out = image.copy()
        density = {1:0.005, 2:0.01, 3:0.02}[level]
        h, w = out.shape[:2]
        n = int(h * w * density)
        rows = np.random.randint(0, h, size=n)
        cols = np.random.randint(0, w, size=n)
        out[rows, cols] = (88, 57, 39)  # brown specks
        # (optional) smear a few specks to simulate streaks:
        return out"""
                #Function to apply salt and pepper noise to an image to simulate dust
        if level == 0:
            self.get_logger().warn("Dust level 0 selected, no processing will be done.")
            return image  #no processing done
        dust = {1:0.01, 2:0.03, 3:0.05}.get(level, 0.01)  #different levels of dustiness

        output = image.copy()
        x, y, ch = output.shape
        noisy_pixels = int(x * y * dust)

        for _ in range(noisy_pixels):
            row, col = np.random.randint(0, x), np.random.randint(0, y)
            output[row, col] = [88, 57, 39]  # brownish color for dust

        return output

    def motion_blur(self, image, level):
        if level == 0:
            self.get_logger().warn("Motion blur level 0 selected, no processing will be done.")
            return image  #no processing done
        blur = {1: 10, 2: 20, 3: 30}.get(level, 10)
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

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
