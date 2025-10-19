import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

class ModelRunnerNode(Node):
    def __init__(self):
        super().__init__('model_runner')

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Publisher of annotated images
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
 
        # Initialise the YOLO model
        model_path = "src/model_runner/models/model_1/my_model.pt"  # Relative to your current working directory        
        self.model = YOLO(model_path)

        # Camera sensor subscription
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 20)

    def image_callback(self, image_msg):
         
        # Turn received image into cv format
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
        # Debug
        self.get_logger().info('Image received from camera')
    
        # Execute computer vision model
        results = self.model(image, stream=True, conf=0.5)  # Configure to detect when confidence > 50%

        if (results):
            self.artifact_found_ = True

        # Process detection
        for result in results:
            # Boxes object for bounding box outputs
            boxes = result.boxes  

            # TODO - Debug: Image size
            original_height, original_width = result.orig_shape  
            self.get_logger().info('image size: %s height by %s width' % (original_height, original_width))

            # Calculate center coordinates
            center_x = int(original_width / 2)
            center_y = int(original_height / 2)

            # Draw circle at image center - Assuming this is the principle axis
            cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1) # red circle with radius 5
            label = "principle axis (Z)"
            cv2.putText(image, label, (center_x, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            # Process any detections if they exist
            number_of_boxes = len(boxes.xywh)
            if number_of_boxes > 0:

                # Get name of detected object
                for box in boxes:
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    self.get_logger().info('Detected Object: "%s"' % class_name)

                # Draw bounding boxes and labels
                for i in range(number_of_boxes):
                    self.get_logger().info('Box x, y, width, height: "%s"' % boxes.xywh[i])

                    x = int(boxes.xywh[i][0])
                    y = int(boxes.xywh[i][1])
                    width = int(boxes.xywh[i][2])
                    height = int(boxes.xywh[i][3])

                    self.get_logger().info('x coordinate of detection on image: "%s"' % x)
                    self.get_logger().info('y coordinate of detection on image: "%s"' % y)
                    self.get_logger().info('width of detection box: "%s"' % width)
                    self.get_logger().info('height of detection box: "%s"' % height)

                    # Draw bounding box
                    cv2.rectangle(image, (x, y), (x + height, y + width), (0, 255, 0), 5)

                    # Add text with class name and confidence score above the bounding box
                    class_id = int(boxes.cls[i])
                    confidence = float(boxes.conf[i])
                    class_name = self.model.names[class_id]
                    label = f"{class_name} {confidence:.2%}"  # Format confidence as percentage
                    cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    # TODO - Debug: Display image on a plot
                    plt.imshow(image, interpolation='none') # Plot the image, turn off interpolation
                    plt.show() # Show the image window
        
        # Re-convert processed cv image to ros format
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")

        # Publish format
        self.image_detections_pub_.publish(image_detection_message)

        # TOOD - Debug: Artificial delay because going to try save some images for investigation
        time.sleep(10)  # Pause for 10 seconds

def main(args=None):
    rclpy.init(args=args)

    # Run indefinitely
    node = ModelRunnerNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()