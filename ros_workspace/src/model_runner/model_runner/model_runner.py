import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
import numpy as np
from ultralytics import YOLO
import cv2

class Artefact:
    """Class for keeping track of detected artefacts"""
    def __init__(self, object_type: str, offset: int):
        self.object_type = object_type      # Class name from YOLO detection
        self.offset = offset                # Pixel offset from center
        self.tracking = False               # Whether this object is currently being tracked
        self.localised = False              # Whether we have determined the object's location
        self.detected_location = None       # Robot's pose when object was detected (Pose2D)
        self.last_seen = 0                  # Frames since last detection
    
    def update_location(self, pose: Pose2D):
        """Update the detected location with current robot pose"""
        self.detected_location = pose
        self.localised = True
    
    def update_offset(self, offset: int):
        """Update the pixel offset from center"""
        self.offset = offset
        self.last_seen = 0  # Reset counter since we've seen it
    
    def increment_last_seen(self):
        """Increment the counter for frames since last detection"""
        self.last_seen += 1

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
        
        # Robot state
        self.current_pose = None        # Current robot pose
        self.frames_without_target = 0  # Count frames where target is lost
        self.max_frames_lost = 10       # Number of frames before considering target lost
        
        # Artefact tracking
        self.current_artefact = None  # Currently tracked artefact
        self.detected_artefacts = {}  # Dictionary to store all detected artefacts {object_type: Artefact}
        
        # Subscribe to RGB camera
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 20)

        # Subscribe to camera intrinsics and extrinsics
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.save_intrinsics, 10)

        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(Pose2D, 'robot_pose', self.pose_callback, 10)

        # Publisher of annotated images
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
    
    def save_intrinsics(self, msg):
        # Guard condition so function only runs once
        if self.has_camera_info:
            return 
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.has_camera_info = True
        
    def pose_callback(self, pose_msg: Pose2D):
        """Update current robot pose and update artefact location if tracking"""
        self.current_pose = pose_msg
        
        # If we're tracking an artefact and it's not yet localised, record its location
        if self.current_artefact and not self.current_artefact.localised:
            self.current_artefact.update_location(pose_msg)
    
    def image_callback(self, image_msg):
         
        # Turn received image into cv format
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
        # Debug
        self.get_logger().info('Image received from camera')
    
        # Execute computer vision model
        results = self.model(image, stream=True, conf=0.5)  # Configure to detect when confidence > 50%

        # Process detection
        for result in results:

            # Debug
            self.get_logger().info('New detection made on received image')    
            
            # Boxes object for bounding box outputs
            boxes = result.boxes  

            # Check if we've lost our target
            self.frames_without_target += 1
            if self.frames_without_target >= self.max_frames_lost and self.target_lock:
                self.target_lock = False
                self.target_object = None
                self.get_logger().warn('Target lost - searching for new target')

            # Calculate center coordinates only if not already set
            original_height, original_width = result.orig_shape 
            if self.center_x is None or self.center_y is None:
                self.center_x = int(original_width / 2)
                self.center_y = int(original_height / 2)
                self.get_logger().info(f'Calculated image center: ({self.center_x}, {self.center_y})')

            # Draw red circle at image center 
            cv2.circle(image, ( self.center_x , self.center_y), 5, (255, 0, 0), -1) 
            cv2.putText(image,"center", ( self.center_x , self.center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            number_of_boxes = len(boxes.xywh)
            
            if number_of_boxes > 0:
                # Reset frame counter since we have detections
                self.frames_without_target = 0
                
                # Find the best detection to track
                best_detection = None
                best_confidence = 0.0
                
                for i in range(number_of_boxes):
                    class_id = int(boxes.cls[i])
                    class_name = self.model.names[class_id]
                    confidence = float(boxes.conf[i])
                    object_image_x = int(boxes.xywh[i][0])
                    object_image_y = int(boxes.xywh[i][1])
                    
                    # If we're not tracking anything yet, or this is our target
                    if (not self.target_lock) or (self.target_object == class_name and confidence > best_confidence):
                        best_detection = {
                            'class_name': class_name,
                            'confidence': confidence,
                            'x': object_image_x,
                            'y': object_image_y,
                            'index': i
                        }
                        best_confidence = confidence
                    
                    # Always draw detection circles (dimmer for non-targets)
                    color = (0, 255, 0) if not self.target_lock else (0, 100, 0)  # Bright green for no target, dim green otherwise
                    cv2.circle(image, (object_image_x, object_image_y), 5, color, -1)
                    
                    # Add labels for all detections
                    label = f"{class_name} {confidence:.2%}"
                    cv2.putText(image, label, (object_image_x, object_image_y - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                
                # Process the best detection
                if best_detection:
                    if not self.target_lock:
                        self.target_object = best_detection['class_name']
                        self.target_confidence = best_detection['confidence']
                        self.target_lock = True
                        self.get_logger().info(f'Locked onto target: {self.target_object}')
                    
                    # Draw arrow and highlight for target object
                    cv2.circle(image, (best_detection['x'], best_detection['y']), 8, (0, 255, 255), 2)  # Yellow highlight
                    cv2.arrowedLine(image, (self.center_x, self.center_y), 
                                  (best_detection['x'], best_detection['y']), 
                                  (0, 0, 255), 2, tipLength=0.3)
                    
                    # Assess how centered we are
                    x_center = self.center_x - best_detection['x']
                    self.get_logger().info(f'Target offset: {x_center} pixels') # If x_center > 50 or < -50 then we are not centered

        # Add status information to top left
        status_y = 30       # Starting y position for text
        line_height = 30    # Pixels between lines
        
        # Tracking status
        tracking_text = f"Tracking: {self.target_object if self.target_lock else 'None'}"
        cv2.putText(image, tracking_text, (10, status_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # Offset information (only show if tracking)
        if self.target_lock and 'x_center' in locals():
            cv2.putText(image, f"Offset: {x_center} px", (10, status_y + line_height), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        else:
            cv2.putText(image, f"No offset", (10, status_y + line_height), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # Frames without target
        cv2.putText(image, f"Frames without target: {self.frames_without_target}", 
                   (10, status_y + 2 * line_height), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

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