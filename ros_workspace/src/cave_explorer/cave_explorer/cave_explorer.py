#!/usr/bin/env python3

import math
import random
from enum import Enum

import cv2  
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from ultralytics import YOLO

def wrap_angle(angle):
    """Function to wrap an angle between 0 and 2*Pi"""
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    """Convert a Pose2D to a full 3D Pose"""
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta / 2.0)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose

class PlannerType(Enum):
    ERROR = 0
    MOVE_FORWARDS = 1
    RETURN_HOME = 2
    GO_TO_FIRST_ARTIFACT = 3
    RANDOM_WALK = 4
    RANDOM_GOAL = 5
    # Add more!

class Artefact:
    def __init__(self, objectType: str, offset: int, tracking: bool, localised: bool, estimated_location: Pose2D, detected_location: Pose2D):
        self.objectType = objectType
        self.offset = offset
        self.tracking = tracking
        self.localised = localised
        self.estimated_location = estimated_location
        self.detected_location = detected_location

class CaveExplorer(Node):
    def __init__(self):
        super().__init__('cave_explorer_node')

        # Variables/Flags for mapping
        self.xlim_ = [0.0, 0.0]
        self.ylim_ = [0.0, 0.0]

        # Variables/Flags for planning
        self.planner_type_ = PlannerType.ERROR
        self.reached_first_artifact_ = False
        self.returned_home_ = False

        # Template code - Publisher for marker artefacts
        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker_array_artifacts', 10)

        # Prepare transformation to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for nav2
        self.nav2_action_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().warn('Waiting for navigate_to_pose action...')
        self.nav2_action_client_.wait_for_server()
        self.get_logger().warn('navigate_to_pose connected')
        self.ready_for_next_goal_ = True
        self.declare_parameter('print_feedback', rclpy.Parameter.Type.BOOL)

        # Publisher for the goal pose visualisation
        self.goal_pose_vis_ = self.create_publisher(PoseStamped, 'goal_pose', 1)

        # Publisher for robot's pose
        self.robot_pose_ = self.create_publisher(Pose2D, 'robot_pose', 1)

        # Subscribe to the map topic to get current bounds
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 1)
        
        # Create a timer to call main_loop periodically (every 1 second)
        self.timer_ = self.create_timer(1.0, self.main_loop)

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Initialise the YOLO model
        model_path = "src/model_runner/models/model_1/my_model.pt"  # Relative to your current working directory        
        self.model = YOLO(model_path)

        # Initiliise image paramters
        self.center_x = 360 # Hardcoded
        self.center_y = 240 # Hardcoded
        
        # Initiliise Robot state
        self.current_pose = Pose2D        
        
        # Initiliise Artefact tracking
        self.artifact_found_ = False
        self.artifact_locations_ = []
        
        # Subscribe to RGB camera
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 20)

        # Publisher of annotated images
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
    
    def get_pose_2d(self):
        """Get the 2d pose of the robot"""

        # Lookup the latest transform
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = t.transform.translation.x
        pose.y = t.transform.translation.y

        qw = t.transform.rotation.w
        qz = t.transform.rotation.z

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        self.get_logger().warn(f'Pose: {pose}')

        self.robot_pose_.publish(pose)
        
        return pose

    def map_callback(self, map_msg: OccupancyGrid):
        """New map received, so update x and y limits"""

        # Extract data from message
        map_origin = [map_msg.info.origin.position.x, 
                      map_msg.info.origin.position.y]
        map_resolution = map_msg.info.resolution
        map_height = map_msg.info.height
        map_width = map_msg.info.width

        # Set current limits
        self.xlim_ = [map_origin[0], map_origin[0]+map_width*map_resolution]
        self.ylim_ = [map_origin[1], map_origin[1]+map_height*map_resolution]

        # self.get_logger().warn('Map received:')
        # self.get_logger().warn(f'  xlim = [{self.xlim_[0]:.2f}, {self.xlim_[1]:.2f}]')
        # self.get_logger().warn(f'  ylim = [{self.ylim_[0]:.2f}, {self.ylim_[1]:.2f}]')
 
    def image_callback(self, image_msg):

        # Only process new detections if artefact has not beet detected
        if self.artifact_found_ == False:

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

                        # Add labels 
                        label = f"{class_name} {confidence:.2%}"
                        cv2.putText(image, label, (object_image_x, object_image_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                        # Create artefact object
                        newArtefact = Artefact(class_name, offset, False, False, Pose2D(), Pose2D())
                        
                        # Debug offset information
                        self.get_logger().info(f'Artefact offset from center: {offset} pixels')

                        # Update detected artefact's parameters
                        newArtefact.detected_location = self.current_pose
                        newArtefact.tracking = True
                        newArtefact.localised = False

                        # TODO Debug - Information on the artefact
                        self.get_logger().info(f'Artefact Type: {newArtefact.objectType}')
                        self.get_logger().info(f'Artefact Offset: {newArtefact.offset}')
                        self.get_logger().info(f'Artefact detection x: {newArtefact.detected_location.x}')
                        self.get_logger().info(f'Artefact detection y: {newArtefact.detected_location.y}')
                        self.get_logger().info(f'Artefact detection theta: {newArtefact.detected_location.theta}')

                        # Append the new artefact object into the artefact list
                        self.artifact_locations_.append(newArtefact)

                        # Change tracking status
                        self.artifact_found_ = True
                                
            # Re-convert processed cv image to ros format
            image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")

            # Publish annotated image back to ros
            self.image_detections_pub_.publish(image_detection_message)
        else:
            return

    def planner_inspection(self, artefact: Artefact):

        # Debug
        self.get_logger().info('Execute inspection of artefact')
        pass

    def localise_artifact(self):
        """
        INCOMPLETE:
        Compute the location of the artifact
        Save it to a list, publish rviz marker
        This version just uses the robot location rather than the artifact location
        You can find other examples of using RViz markers in the previous assignments template code
        """

        # Current location of the robot
        robot_pose = self.get_pose_2d()

        if robot_pose == None:
            self.get_logger().warn(f'localise_artifact: robot_pose is None.')
            return

        # Compute the location of the artifact
        # This is currently INCOMPLETE
        point = Point()
        point.x = robot_pose.x
        point.y = robot_pose.y
        point.z = 1.0

        # Save it
        self.artifact_locations_.append(point)

        # Publish the markers
        self.publish_artifact_markers()

    def publish_artifact_markers(self):
        """ Publish the artifact location markers"""

        # Update the locations
        self.marker_artifacts_.points = self.artifact_locations_

        # Create and publish the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [self.marker_artifacts_]
        self.marker_pub_.publish(marker_array)

    def planner_go_to_pose2d(self, pose2d):
        """Go to a provided 2d pose"""

        # Send a goal to navigate_to_pose with self.nav2_action_client_
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.header.frame_id = 'map'
        action_goal.pose.pose = pose2d_to_pose(pose2d)

        # Publish visualisation
        self.goal_pose_vis_.publish(action_goal.pose)

        # Decide whether to show feedback or not
        if self.get_parameter('print_feedback').value:
            feedback_method = self.feedback_callback
        else:
            feedback_method = None

        # Send goal to action server
        self.get_logger().warn(f'Sending goal [{pose2d.x:.2f}, {pose2d.y:.2f}]...')
        self.send_goal_future_ = self.nav2_action_client_.send_goal_async(
            action_goal,
            feedback_callback=feedback_method)
        self.send_goal_future_.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """The requested goal pose has been sent to the action server"""

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        # Goal accepted: get result when it's completed
        self.get_logger().warn(f'Goal accepted')
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.goal_reached_callback)

    def feedback_callback(self, feedback_msg):
        """Monitor the feedback from the action server"""

        feedback = feedback_msg.feedback

        self.get_logger().info(f'{feedback.distance_remaining:.2f} m remaining')

    def goal_reached_callback(self, future):
        """The requested goal has been reached"""

        result = future.result().result
        self.get_logger().info(f'Goal reached!')
        self.ready_for_next_goal_ = True

    def planner_move_forwards(self, distance):
        """Simply move forward by the specified distance"""

        pose_2d = self.get_pose_2d()

        pose_2d.x += distance * math.cos(pose_2d.theta)
        pose_2d.y += distance * math.sin(pose_2d.theta)

        self.planner_go_to_pose2d(pose_2d)

    def planner_go_to_first_artifact(self):
        """Go to a pre-specified artifact location"""

        goal_pose2d = Pose2D(
            x = 18.1,
            y = 6.6,
            theta = math.pi/2
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_return_home(self):
        """Return to the origin"""

        goal_pose2d = Pose2D(
            x = 0.0,
            y = 0.0,
            theta = math.pi
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_random_walk(self):
        """Go to a random location, which may be invalid"""

        # Select a random location
        goal_pose2d = Pose2D(
            x = random.uniform(self.xlim_[0], self.xlim_[1]),
            y = random.uniform(self.ylim_[0], self.ylim_[1]),
            theta = random.uniform(0, 2*math.pi)
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_random_goal(self):
        """Go to a random location out of a predefined set"""

        # Hand picked set of goal locations
        random_goals = [[15.2, 2.2],
                        [30.7, 2.2],
                        [43.0, 11.3],
                        [36.6, 21.9],
                        [33.0, 30.4],
                        [40.4, 44.3],
                        [51.5, 37.8],
                        [16.0, 24.1],
                        [3.4, 33.5],
                        [7.9, 13.8],
                        [14.2, 37.7]]

        # Select a random location
        goal_valid = False
        while not goal_valid:
            idx = random.randint(0,len(random_goals)-1)
            goal_x = random_goals[idx][0]
            goal_y = random_goals[idx][1]

            # Only accept this goal if it's within the current costmap bounds
            if goal_x > self.xlim_[0] and goal_x < self.xlim_[1] and \
               goal_y > self.ylim_[0] and goal_y < self.ylim_[1]:
                goal_valid = True
            else:
                self.get_logger().warn(f'Goal [{goal_x}, {goal_y}] out of bounds')

        goal_pose2d = Pose2D(
            x = goal_x,
            y = goal_y,
            theta = random.uniform(0, 2*math.pi)
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def main_loop(self):
        """
        Set the next goal pose and send to the action server
        See https://docs.nav2.org/concepts/index.html
        """
        # Run pose
        self.get_pose_2d()
    
        # Debug - State of tracking
        self.get_logger().info(f"State of tracking: {self.artifact_found_}")

        # Debug - Artefact list
        self.get_logger().info('Current artefacts:')
        for item in self.artifact_locations_:
            self.get_logger().info(f'Artefact: Type={item.objectType}, Offset={item.offset}, Tracking={item.tracking}, Localised={item.localised}')

        # Don't do anything until SLAM is launched
        if not self.tf_buffer.can_transform(
                'map',
                'base_link',
                rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform... Have you launched a SLAM node?')
            return

        #######################################################
        # Update flags related to the progress of the current planner

        # Check if previous goal still running
        if not self.ready_for_next_goal_:
            self.get_logger().info(f'Previous goal still running')
            return

        self.ready_for_next_goal_ = False

        if self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.get_logger().info('Successfully reached first artifact!')
            self.reached_first_artifact_ = True
        if self.planner_type_ == PlannerType.RETURN_HOME:
            self.get_logger().info('Successfully returned home!')
            self.returned_home_ = True

        #######################################################
        # Select the next planner to execute
        # Update this logic as you see fit!
        if not self.reached_first_artifact_:
            self.planner_type_ = PlannerType.GO_TO_FIRST_ARTIFACT
        elif not self.returned_home_:
            self.planner_type_ = PlannerType.RETURN_HOME
        else:
            self.planner_type_ = PlannerType.RANDOM_GOAL

        #######################################################
        # Execute the planner by calling the relevant method
        # Add your own planners here!
        self.get_logger().info(f'Calling planner: {self.planner_type_.name}')
        if self.planner_type_ == PlannerType.MOVE_FORWARDS:
            self.planner_move_forwards(10)
        elif self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.planner_go_to_first_artifact()
        elif self.planner_type_ == PlannerType.RETURN_HOME:
            self.planner_return_home()
        elif self.planner_type_ == PlannerType.RANDOM_WALK:
            self.planner_random_walk()
        elif self.planner_type_ == PlannerType.RANDOM_GOAL:
            self.planner_random_goal()
        else:
            self.get_logger().error('No valid planner selected')
            self.destroy_node()
        #######################################################

def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    while rclpy.ok():
        rclpy.spin(cave_explorer)