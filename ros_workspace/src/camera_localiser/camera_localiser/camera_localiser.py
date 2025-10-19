import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

class CameraLocaliserNode(Node):
    def __init__(self):
        super().__init__('camera_localiser')

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()
        
        # Initialise camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.projection_matrix = None
        self.has_camera_info = False

        # Create subscriptions
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.model_detection_sub = self.create_subscription(Image, 'detections_image', self.localise_coordinates, 10)

    def camera_info_callback(self, msg):
        if self.has_camera_info:
            return  # Already received camera info

        # Extract camera matrix (K)
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        
        # Extract distortion coefficients
        self.dist_coeffs = np.array(msg.d)
        
        # Extract projection matrix (P)
        self.projection_matrix = np.array(msg.p).reshape(3, 4)
        
        self.has_camera_info = True
        self.get_logger().info('Received camera calibration parameters')
        
        # Log camera details in a readable format
        logger = self.get_logger()
        logger.info('Camera Details:')
        logger.info('-' * 50)
        
        # Log basic camera info
        logger.info(f'Frame ID: {msg.header.frame_id}')
        logger.info(f'Image Height: {msg.height}')
        logger.info(f'Image Width: {msg.width}')
        
        # Log camera matrix (intrinsic parameters)
        logger.info('\nCamera Matrix (K):')
        for row in self.camera_matrix:
            logger.info(f'{row}')
            
        # Log distortion coefficients
        logger.info('\nDistortion Coefficients:')
        logger.info(f'{self.dist_coeffs}')
        
        # Log projection matrix
        logger.info('\nProjection Matrix (P):')
        for row in self.projection_matrix:
            logger.info(f'{row}')
            
        # Log ROI information if available
        if msg.roi.do_rectify:
            logger.info('\nROI Information:')
            logger.info(f'ROI x_offset: {msg.roi.x_offset}')
            logger.info(f'ROI y_offset: {msg.roi.y_offset}')
            logger.info(f'ROI height: {msg.roi.height}')
            logger.info(f'ROI width: {msg.roi.width}')
        
        logger.info('-' * 50)
        
    def localise_coordinates(self, Image):
        self.get_logger().info('Received object detection')

def main(args=None):
    rclpy.init(args=args)

    # Run indefinitely
    node = CameraLocaliserNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()