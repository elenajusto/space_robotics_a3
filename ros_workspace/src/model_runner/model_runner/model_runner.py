import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D, PoseStamped
import numpy as np
from ultralytics import YOLO
import cv2

class Artefact:
    def __init__(self, objectType: str, offset: int, tracking: bool, localised: bool, estimated_location: Pose2D, detected_location: Pose2D):
        self.objectType = objectType
        self.offset = offset
        self.tracking = tracking
        self.localised = localised
        self.estimated_location = estimated_location
        self.detected_location = detected_location

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
        self.center_x = 360 # Hardcoded
        self.center_y = 240 # Hardcoded
        
        # Initiliise Robot state
        self.current_pose = Pose2D        
        
        # Initiliise Artefact tracking
        self.artefact_detected = False      # will serve as tracking flag
        self.artefact_list = None
        
        # Subscribe to RGB camera
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 20)

        # Subscribe to camera intrinsics and extrinsics
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.save_intrinsics, 10)

        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(Pose2D, '/robot_pose', self.pose_callback, 10)

        # Publisher of annotated images
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
    
    def save_intrinsics(self, msg):
        # Guard condition so function only runs once
        if self.has_camera_info:
            return 
        else:
            # Debug
            self.get_logger().info('Received camera intrinsics')
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.has_camera_info = True
        
    def pose_callback(self, pose_msg):
        # Debug
        self.get_logger().info('Received robot pose')
        self.current_pose = pose_msg
    
    def image_callback(self, image_msg):
        
        # Debug
        self.get_logger().info('Image received from camera')
    
        # Turn received image into cv format
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
         # Draw red circle at image center 
        cv2.circle(image, ( self.center_x , self.center_y), 5, (255, 0, 0), -1) 
        cv2.putText(image,"center", ( self.center_x , self.center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

        # Execute computer vision model
        results = self.model(image, stream=True, conf=0.5)  # Configure to detect when confidence > 50%

        # Process detection
        for result in results:
            # Process computer vision model results
            boxes = result.boxes                            
            number_of_boxes = len(boxes.xywh)
            if number_of_boxes > 0:

                # Process given box and allocate to an artefact
                for i in range(number_of_boxes):
                    class_id = int(boxes.cls[i])
                    class_name = self.model.names[class_id]
                    confidence = float(boxes.conf[i])
                    object_image_x = int(boxes.xywh[i][0])
                    object_image_y = int(boxes.xywh[i][1])
                    
                    # Create dot on detected object
                    cv2.circle(image, (object_image_x, object_image_y), 5, (0, 255, 0), -1)
                    
                    # Draw arrow from center of camera to detected object
                    cv2.arrowedLine(image, (self.center_x, self.center_y), (object_image_x, object_image_y), (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.2) 
                    
                    # Get offset between image center and artefact (horizontal distance in pixels)
                    offset = object_image_x - self.center_x
                    
                    # Display offset near the middle of the arrow
                    mid_x = (self.center_x + object_image_x) // 2
                    mid_y = (self.center_y + object_image_y) // 2
                    cv2.putText(image, f"offset: {offset}px", (mid_x, mid_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # Add labels 
                    label = f"{class_name} {confidence:.2%}"
                    cv2.putText(image, label, (object_image_x, object_image_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    # Create artefact object
                    newArtefact = Artefact(class_name, offset, False, False, Pose2D(), Pose2D())
                    
                    # Debug offset information
                    self.get_logger().info(f'Artefact offset from center: {offset} pixels')

                    self.inspect(newArtefact)
                    
        # Re-convert processed cv image to ros format
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")

        # Publish annotated image back to ros
        self.image_detections_pub_.publish(image_detection_message)

    def inspect(self, artefact: Artefact):

        # Update detected artefact's parameters
        artefact.detected_location = self.current_pose
        artefact.tracking = True
        artefact.localised = False

        # Debug Information on the artefact
        self.get_logger().info(f'Artefact Type: {artefact.objectType}')
        self.get_logger().info(f'Artefact Offset: {artefact.offset}')
        self.get_logger().info(f'Artefact detection x: {artefact.detected_location.x}')
        self.get_logger().info(f'Artefact detection y: {artefact.detected_location.y}')
        self.get_logger().info(f'Artefact detection theta: {artefact.detected_location.theta}')

def main(args=None):
    rclpy.init(args=args)

    # Run indefinitely
    node = ModelRunnerNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()