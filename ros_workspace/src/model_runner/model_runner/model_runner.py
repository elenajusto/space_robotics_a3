import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import cv2

class ModelRunnerNode(Node):
    def __init__(self):
        super().__init__('model_runner')

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Initialise the YOLO model
        model_path = "src/model_runner/models/model_1/my_model.pt"  # Relative to your current working directory        
        self.model = YOLO(model_path)

        # Initialise camera parameters
        self.camera_matrix = None
        self.has_camera_info = False

        # Initiliise image paramters
        self.center_x = None
        self.center_y = None
        
        # Subscribe to RGB camera
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 20)

        # Subscribe to camera intrinsics and extrinsics
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.save_intrinsics, 10)

        # Publisher of annotated images
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
    
    def save_intrinsics(self, msg):
        # Guard condition so function only runs once
        if self.has_camera_info:
            return 
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.has_camera_info = True
    
    def image_callback(self, image_msg):
         
        # Turn received image into cv format
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
        # Debug
        self.get_logger().info('Image received from camera')
    
        # Execute computer vision model
        results = self.model(image, stream=True, conf=0.5)  # Configure to detect when confidence > 50%

        # Process detection
        for result in results:
            
            # Boxes object for bounding box outputs
            boxes = result.boxes  

            # Get center point if not already set
            original_height, original_width = result.orig_shape  
            self.get_logger().info('image size: %s height by %s width' % (original_height, original_width))

            # Calculate center coordinates only if not already set
            if self.center_x is None or self.center_y is None:
                self.center_x = int(original_width / 2)
                self.center_y = int(original_height / 2)
                self.get_logger().info(f'Calculated image center: ({self.center_x}, {self.center_y})')

            # Draw circle at image center 
            cv2.circle(image, ( self.center_x , self.center_y), 5, (255, 0, 0), -1) 
            cv2.putText(image,"center", ( self.center_x , self.center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

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

                    object_image_x = int(boxes.xywh[i][0])
                    object_image_y = int(boxes.xywh[i][1])
    
                    self.get_logger().info('x coordinate of detection on image: "%s"' % object_image_x)
                    self.get_logger().info('y coordinate of detection on image: "%s"' % object_image_y)
              
                    # Draw circle at detection point (green to match the label)
                    cv2.circle(image, (object_image_x, object_image_y), 5, (0, 255, 0), -1)

                    # Add text with class name and confidence score above detection point
                    class_id = int(boxes.cls[i])
                    confidence = float(boxes.conf[i])
                    class_name = self.model.names[class_id]
                    label = f"{class_name} {confidence:.2%}"  
                    cv2.putText(image, label, (object_image_x, object_image_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    # Draw an arrow from center to detection point
                    cv2.arrowedLine(image, (self.center_x, self.center_y), (object_image_x, object_image_y), (0, 0, 255), 2, tipLength=0.3)  
        
        # Re-convert processed cv image to ros format
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")

        # Publish annotated image back to ros
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