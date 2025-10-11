import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import numpy as np
import os

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

        # Test subscription
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

        # Camera sensor subscription
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 20)

    def listener_callback(self, msg):
        # As a test listen to our minal publisher
        self.get_logger().info('I heard: "%s"' % msg.data)

    def image_callback(self, msg):
        # Test listen for images
        image = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Debug
        self.get_logger().info('Image detected')

        # Call the yolo model on the image
        self.execute_model(image)

    def execute_model(self, image):
        results = self.model(image, stream=True)

        for result in results:

            # Boxes object for bounding box outputs
            boxes = result.boxes  

            # Get name of detected object
            for box in boxes:
                class_id = int(box.cls)
                class_name = self.model.names[class_id]
                self.get_logger().info('class_name: "%s"' % class_name)

            # Get bounding box of detected object
            number_of_boxes = len(boxes.xywh)
            if number_of_boxes > 0:
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

                    # Publish the image with the detection bounding boxes and labels
                    image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")
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