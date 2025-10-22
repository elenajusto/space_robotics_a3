import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import numpy as np

class ModelRunnerNode(Node):
    def __init__(self):
        super().__init__('model_runner')

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Publisher
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

            # Process any detections if they exist
            number_of_boxes = len(boxes.xywh)
            if number_of_boxes > 0:
                # Get name of detected object
                for box in boxes:
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    self.get_logger().info('class_name: "%s"' % class_name)

                # Draw bounding boxes and labels
                for i in range(number_of_boxes):
                    self.get_logger().info('box: "%s"' % boxes.xywh[i])

                    x = int(boxes.xywh[i][0])
                    y = int(boxes.xywh[i][1])
                    width = int(boxes.xywh[i][2])
                    height = int(boxes.xywh[i][3])

                    self.get_logger().info('x: "%s"' % x)
                    self.get_logger().info('y: "%s"' % y)
                    self.get_logger().info('width: "%s"' % width)
                    self.get_logger().info('height: "%s"' % height)

                    # Draw bounding box
                    cv2.rectangle(image, (x, y), (x + height, y + width), (0, 255, 0), 5)

                    # Add text with class name and confidence score above the bounding box
                    class_id = int(boxes.cls[i])
                    confidence = float(boxes.conf[i])
                    class_name = self.model.names[class_id]
                    label = f"{class_name} {confidence:.2%}"  # Format confidence as percentage
                    cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Re-convert processed cv image to ros format
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")

        # Publish format
        self.image_detections_pub_.publish(image_detection_message)

def main(args=None):
    rclpy.init(args=args)

    # Run indefinitely
    node = ModelRunnerNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()