import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ultralytics import YOLO
import cv2
import numpy as np
import os

class ModelRunnerNode(Node):
    def __init__(self):
        super().__init__('model_runner')
        self.get_logger().info('ModelRunner started')
        
        # Debug to see where Python is looking for files
        current_dir = os.getcwd()
        self.get_logger().info(f"Current working directory: {current_dir}")
        self.get_logger().info("Directory contents:")
        for item in os.listdir(current_dir):
            self.get_logger().info(f"- {item}")
        
        # Init the YOLO model
        try:
            # Initialize the YOLO model with explicit path
            model_path = "src/model_runner/models/model_1/my_model.pt"  # Relative to your current working directory
            self.get_logger().info(f"Attempting to load model from: {model_path}")
            
            self.model = YOLO(model_path)
            #self.get_logger().info(f"Model loaded successfully: {self.model}") # This will print out the entire model

            self.get_logger().info(f"Model loaded successfully.") # Just a debug statement
            
            # Test if model attributes are accessible
            self.get_logger().info(f"Model task type: {self.model.task}")
            self.get_logger().info(f"Model names: {self.model.names if hasattr(self.model, 'names') else 'No names available'}")
        except Exception as e:
            self.get_logger().error(f'Error loading model: {str(e)}')
            raise 

        # Subscriptions
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        # As a test listen to our minal publisher
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    # Run indefinitely
    node = ModelRunnerNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()