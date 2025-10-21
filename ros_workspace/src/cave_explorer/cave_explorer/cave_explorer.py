#!/usr/bin/env python3

import math
import random
from enum import Enum
import numpy as np
from scipy.ndimage import label, center_of_mass

import cv2  # OpenCV2
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

    FRONTIER_EXPLORATION = 6


class CaveExplorer(Node):
    def __init__(self):
        super().__init__('cave_explorer_node')

        # Variables/Flags for mapping
        self.xlim_ = [0.0, 0.0]
        self.ylim_ = [0.0, 0.0]

        # Variables/Flags for perception
        self.artifact_found_ = False

        # Variables/Flags for planning
        self.planner_type_ = PlannerType.ERROR
        self.reached_first_artifact_ = False
        self.returned_home_ = False

        # Marker for artifact locations
        # See https://wiki.ros.org/rviz/DisplayTypes/Marker
        self.marker_artifacts_ = Marker()
        self.marker_artifacts_.header.frame_id = "map"
        self.marker_artifacts_.ns = "artifacts"
        self.marker_artifacts_.id = 0
        self.marker_artifacts_.type = Marker.SPHERE_LIST
        self.marker_artifacts_.action = Marker.ADD
        self.marker_artifacts_.pose.position.x = 0.0
        self.marker_artifacts_.pose.position.y = 0.0
        self.marker_artifacts_.pose.position.z = 0.0
        self.marker_artifacts_.pose.orientation.x = 0.0
        self.marker_artifacts_.pose.orientation.y = 0.0
        self.marker_artifacts_.pose.orientation.z = 0.0
        self.marker_artifacts_.pose.orientation.w = 1.0
        self.marker_artifacts_.scale.x = 1.5
        self.marker_artifacts_.scale.y = 1.5
        self.marker_artifacts_.scale.z = 1.5
        self.marker_artifacts_.color.a = 1.0
        self.marker_artifacts_.color.r = 0.0
        self.marker_artifacts_.color.g = 1.0
        self.marker_artifacts_.color.b = 0.2
        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker_array_artifacts', 10)

        self.inspected_marker_artifacts_ = Marker()
        self.inspected_marker_artifacts_.header.frame_id = "map"
        self.inspected_marker_artifacts_.ns = "inspected_artifacts"
        self.inspected_marker_artifacts_.id = 0
        self.inspected_marker_artifacts_.type = Marker.SPHERE_LIST
        self.inspected_marker_artifacts_.action = Marker.ADD
        self.inspected_marker_artifacts_.pose.position.x = 0.0
        self.inspected_marker_artifacts_.pose.position.y = 0.0
        self.inspected_marker_artifacts_.pose.position.z = 0.0
        self.inspected_marker_artifacts_.pose.orientation.x = 0.0
        self.inspected_marker_artifacts_.pose.orientation.y = 0.0
        self.inspected_marker_artifacts_.pose.orientation.z = 0.0
        self.inspected_marker_artifacts_.pose.orientation.w = 1.0
        self.inspected_marker_artifacts_.scale.x = 1.5
        self.inspected_marker_artifacts_.scale.y = 1.5
        self.inspected_marker_artifacts_.scale.z = 1.5
        self.inspected_marker_artifacts_.color.a = 1.0
        self.inspected_marker_artifacts_.color.r = 1.0
        self.inspected_marker_artifacts_.color.g = 0.0
        self.inspected_marker_artifacts_.color.b = 0.2


        # Remember the artifact locations
        # Array of type geometry_msgs.Point
        self.artifact_locations_ = []

        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

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

        # Subscribe to the map topic to get current bounds
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 1)

        # Prepare image processing and subscribe to image detection to get artifact information
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
        self.declare_parameter('computer_vision_model_filename', rclpy.Parameter.Type.STRING)
        self.computer_vision_model_ = cv2.CascadeClassifier(self.get_parameter('computer_vision_model_filename').value)
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.main_loop)

        # Parameter for navigation type
        self.declare_parameter('planner_type', 'frontier_exploration')
        planner_value = self.get_parameter('planner_type').value
        self.get_logger().info(f'Planner parameter initialised as: {planner_value}')

        self.current_goal_ = None
        self.goal_timeout_sec_ = 8 # Time in seconds rover will spend going to each goal
        self.goal_start_time_ = None
        self.min_unknown_cell_clusters = 16 # Size of grouped consective cells to be valid frontier goal

        # Goal timeout timer
        self.goal_timeout_timer_ = self.create_timer(0.5, self.check_goal_timeout)

        # Behaviour control for state machine
        self.current_behavior_ = "exploration"   # "exploration" or "inspection"
        self.artifact_found_ = False
        self.inspection_goal_sent_ = False
        self.inspected_artifacts_ = []  # store inspected artifact map coords
        self.selected_artifact_ = None  # currently targeted artifact
        self.standoff_distance_ = 3  # distance to stay away from artifact 
        self.inspection_duplicate_distance_ = 1.0  # don't inspect same artifact if within this distance

        # Stores the active nav2 goal handle so we can cancel
        self.current_goal_handle_ = None

        # Timer handle for inspection pause
        self.inspection_pause_timer_ = None

        # depth camera subscription - may be unused until you implement depth processing
        self.depth_image_sub_ = self.create_subscription(Image, 'camera/depth/image', self.depth_image_callback, 1)    




    
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

        # self.get_logger().warn(f'Pose: {pose}')

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

            # Extract map info
        self.map_origin_ = map_msg.info.origin
        self.map_resolution_ = map_msg.info.resolution
        self.map_height_ = map_msg.info.height
        self.map_width_ = map_msg.info.width
        self.map_data_ = map_msg.data  # store the occupancy grid values


        # self.get_logger().warn('Map received:')
        # self.get_logger().warn(f'  xlim = [{self.xlim_[0]:.2f}, {self.xlim_[1]:.2f}]')
        # self.get_logger().warn(f'  ylim = [{self.ylim_[0]:.2f}, {self.ylim_[1]:.2f}]')
    
    def image_callback(self, image_msg):
        """
        Recieve an RGB image.
        Use this method to detect artifacts of interest.
        
        A simple method has been provided to begin with for detecting stop signs (which is not what we're actually looking for) 
        adapted from: https://www.geeksforgeeks.org/detect-an-object-with-opencv-python/
        """
    
        # Copy the image message to a cv image
        # see http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        # Create a grayscale version (some simple models use this)
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Retrieve the pre-trained model
        stop_sign_model = self.computer_vision_model_

        # Detect artifacts in the image
        # The minSize is used to avoid very small detections that are probably noise
        detections = stop_sign_model.detectMultiScale(image, minSize=(20,20))

        # You can set "artifact_found_" to true to signal to "main_loop" that you have found a artifact
        # You may want to communicate more information
        # Since the "image_callback" and "main_loop" methods can run at the same time you should protect any shared variables
        # with a mutex
        # "artifact_found_" doesn't need a mutex because it's an atomic
        num_detections = len(detections)
        

        if num_detections > 0:
            self.artifact_found_ = True
        else:
            self.artifact_found_ = False

        # Draw a bounding box rectangle on the image for each detection
        for(x, y, width, height) in detections:
            cv2.rectangle(image, (x, y), (x + height, y + width), (0, 255, 0), 5)

        # Publish the image with the detection bounding boxes
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")
        self.image_detections_pub_.publish(image_detection_message)

        #If an artifact is found, switch to inspection mode
        if self.artifact_found_ and self.current_behavior_ != "inspection":
            self.get_logger().info('Artifact found!')
            self.get_logger().warn(f'Detection: {detections}')
            self.localise_artifact()
    
    

    # def plan_inspection_goal(self):
    #     """Generate and send a close-range (standoff) goal near the selected artifact."""
    #     if not self.selected_artifact_:
    #         self.get_logger().warn("plan_inspection_goal: no selected artifact.")
    #         self.current_behavior_ = "exploration"
    #         return

    #     artifact_point = self.selected_artifact_
    #     robot_pose = self.get_pose_2d()
    #     if robot_pose is None:
    #         self.get_logger().warn("No robot pose available for inspection.")
    #         self.current_behavior_ = "exploration"
    #         return

    #     dx = artifact_point.x 
    #     dy = artifact_point.y
    #     angle = math.atan2(dy, dx)

    #     goal_x = artifact_point.x - self.standoff_distance_ * math.cos(angle)
    #     goal_y = artifact_point.y - self.standoff_distance_ * math.sin(angle)
    #     goal_yaw = angle

    #     self.get_logger().warn(f"Inspection goal: ({goal_x:.2f}, {goal_y:.2f}), yaw={goal_yaw:.2f}")
    #     goal_pose = Pose2D(x=goal_x, y=goal_y, theta=goal_yaw)

    #     # switch behavior and send the goal using existing planner function
    #     self.current_behavior_ = "inspection"
    #     self.inspection_goal_sent_ = True
    #     self.planner_go_to_pose2d(goal_pose)

    def plan_inspection_goal(self):
        """Generate and send a close-range (standoff) goal near the selected artifact."""
        if not self.selected_artifact_:
            self.get_logger().warn("plan_inspection_goal: no selected artifact Switching back to exploration.")
            self.current_behavior_ = "exploration"
            return

        artifact_point = self.selected_artifact_
        robot_pose = self.get_pose_2d()
        if robot_pose is None:
            self.get_logger().warn("No robot pose available for inspection. Switching back to exploration")
            self.current_behavior_ = "exploration"
            return

        dx = artifact_point.x #- robot_pose.x
        dy = artifact_point.y #- robot_pose.y
        angle = math.atan2(dy, dx)

        goal_x = artifact_point.x - self.standoff_distance_ * math.cos(angle)
        goal_y = artifact_point.y - self.standoff_distance_ * math.sin(angle)
        goal_yaw = angle

        self.get_logger().warn(f"Inspection goal: ({goal_x:.2f}, {goal_y:.2f}), yaw={goal_yaw:.2f}")
        goal_pose = Pose2D(x=goal_x, y=goal_y, theta=goal_yaw)

        # send the goal using existing planner function
        self.inspection_goal_sent_ = True

        # If currently travelling to a frontier goal, cancel it so we can preempt
        if not self.ready_for_next_goal_:
            self.get_logger().info("Cancelling current frontier goal for inspection...")
            self.cancel_current_goal()

        # force sending the inspection goal even if ready flag was false momentarily
        self.planner_go_to_pose2d(goal_pose, force=True)

    def cancel_current_goal(self):
        """Cancel the currently-active navigation goal (if any)."""
        try:
            if self.current_goal_handle_ is not None:
                self.get_logger().info("Sending cancel request for current goal...")
                # goal_handle has cancel_goal_async()
                cancel_future = self.current_goal_handle_.cancel_goal_async()
                # best effort: attach a done callback to log result (not required)
                def _on_cancel_done(fut):
                    try:
                        res = fut.result()
                        self.get_logger().info(f"Cancel request completed: {res}")
                    except Exception as e:
                        self.get_logger().warn(f"Cancel request failed: {e}")
                cancel_future.add_done_callback(_on_cancel_done)
                # clear stored handle now (we are preempting)
                self.current_goal_handle_ = None
                # ensure ready so we can send new goal
                self.ready_for_next_goal_ = True
        except Exception as e:
            self.get_logger().warn(f"Exception while cancelling goal: {e}")
            self.current_goal_handle_ = None
            self.ready_for_next_goal_ = True


    def handle_artifact_goal(self, success: bool):
        """Called when an inspection attempt completes or fails."""
        if self.current_behavior_ != "inspection":
            return

        if success:
            self.get_logger().info("Reached artifact of interest.")
            # We will actually mark the artifact inspected when the pause timer fires.
            # (resume_after_inspection will publish markers and clear state)
            # create a one-shot 3s timer (we cancel it immediately inside callback)
            # ensure no existing timer
            if self.inspection_pause_timer_ is not None:
                try:
                    self.inspection_pause_timer_.cancel()
                except Exception:
                    pass
            # create timer — callback cancels it immediately and resumes
            self.get_logger().info("Inspecting artifact...")
            self.inspection_pause_timer_ = self.create_timer(3.0, self.resume_after_inspection)
        else:
            self.get_logger().warn("Inspection failed. Abandoning this artifact.")
            self.reset_to_exploration()


    def resume_after_inspection(self):
        """Resume normal exploration after inspection pause."""
        # cancel timer so it's not repeated
        try:
            if self.inspection_pause_timer_ is not None:
                self.inspection_pause_timer_.cancel()
        except Exception:
            pass
        self.get_logger().info("Inspection complete. Marking artifact inspected and resuming exploration.")

        # Mark artifact inspected (avoid duplicates)
        if self.selected_artifact_ is not None:
            self.inspected_artifacts_.append(self.selected_artifact_)

        # publish updated markers
        self.publish_inspected_artifact_markers()
        # remove it from artifact_locations_ if present
        try:
            if self.selected_artifact_ in self.artifact_locations_:
                self.artifact_locations_.remove(self.selected_artifact_)
        except ValueError:
            pass

        # reset state
        self.selected_artifact_ = None
        self.inspection_goal_sent_ = False
        self.artifact_found_ = False
        self.current_behavior_ = "exploration"
        self.ready_for_next_goal_ = True
        self.goal_start_time_ = None
        self.inspection_pause_timer_ = None

    def reset_to_exploration(self):
        """Reset inspection-related flags and resume exploration."""
        self.selected_artifact_ = None
        self.inspection_goal_sent_ = False
        self.artifact_found_ = False
        self.current_behavior_ = "exploration"
        self.ready_for_next_goal_ = True
        self.goal_start_time_ = None



    def depth_image_callback(self, depth_image_msg):
        """
        Recieve a depth image.
        Use this method to help localise artifacts of interest.
        """
        # Turn received image into cv format
        depth_image = self.cv_bridge_.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
            
        pass
        # Process depth image here
        # Currently not implemented     
    

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
        # point.x = robot_pose.x + 2 
        # point.y = robot_pose.y + 2
        point.x = 18.1 # Forcing a fake location for artifacts
        point.y = 6.6 
        point.z = 1.0
        

        skip_artifact = False

        # check duplicates to not inspect artifatcs already inspected
        for a in self.inspected_artifacts_:
            # if math.hypot(a.x - point.x, a.y - point.y) < self.inspection_duplicate_distance_:
            if point == a:
                self.get_logger().info("Detected artifact already inspected. Skipping artifact.")  
                self.publish_inspected_artifact_markers()
                skip_artifact = True
                return

        #Will add to locations
        for a in self.artifact_locations_:
            # if math.hypot(a.x - point.x, a.y - point.y) < self.inspection_duplicate_distance_:
            if point == a:
                self.get_logger().info("Artifact already recorded. Skipping artifact.")
                skip_artifact = True
                return

        # Save approx artifact location and publish markers
        
        self.publish_artifact_markers()
        

        # select this artifact to inspect and plan approach
        self.selected_artifact_ = point

        if not skip_artifact:
            #Setting behavious as inspection as artifact should be inspected
            self.artifact_locations_.append(point)
            self.current_behavior_ = "inspection"
            self.get_logger().info("New artifact detect, switching to inspection mode.")
            self.plan_inspection_goal()


    def publish_artifact_markers(self):
        """ Publish the artifact location markers"""

        # Update the locations
        self.marker_artifacts_.points = self.artifact_locations_

        # Create and publish the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [self.marker_artifacts_]
        self.marker_pub_.publish(marker_array)
        self.publish_inspected_artifact_markers()
        
        
    
    def publish_inspected_artifact_markers(self): # Inspected artifacts will be Red
        """ Publish the artifact location markers"""

        # Update the locations
        self.inspected_marker_artifacts_.points = self.inspected_artifacts_

        # Create and publish the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [self.inspected_marker_artifacts_]
        self.marker_pub_.publish(marker_array)

    def planner_go_to_pose2d(self, pose2d, force: bool = False):
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
        if self.ready_for_next_goal_ or force:
            self.get_logger().warn(f'Sending goal [{pose2d.x:.2f}, {pose2d.y:.2f}]...')
            self.send_goal_future_ = self.nav2_action_client_.send_goal_async(
                action_goal,
                feedback_callback=feedback_method)
            self.send_goal_future_.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """The requested goal pose has been sent to the action server"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal response exception: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        # store handle so we can cancel later if needed
        self.current_goal_handle_ = goal_handle

        # Goal accepted: get result when it's completed
        self.get_logger().warn('Goal accepted')
        self.goal_start_time_ = self.get_clock().now()
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.goal_reached_callback)
        self.ready_for_next_goal_ = False



    def feedback_callback(self, feedback_msg):
        """Monitor the feedback from the action server"""

        feedback = feedback_msg.feedback

        self.get_logger().info(f'{feedback.distance_remaining:.2f} m remaining')

    def goal_reached_callback(self, future):
        """The requested goal has been reached"""
        try:
            result = future.result().result
        except Exception:
            result = None

        self.get_logger().info('Goal reached!')
        # clear stored handle
        self.current_goal_handle_ = None

        # If inspecting, start inspection pause (non-blocking) and then mark inspected in resume_after_inspection
        if self.current_behavior_ == "inspection":
            self.get_logger().info("Reached inspection standoff. Starting inspection pause.")
            # call handler which will set a timer for 3s
            self.handle_artifact_goal(success=True)
            # don't set ready_for_next_goal_ True yet — resume_after_inspection will do that
            self.goal_start_time_ = None
        else:
            # normal exploration goal finished; continue
            self.current_goal_ = None
            self.ready_for_next_goal_ = True
            self.goal_start_time_ = None




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

        """Main decision loop"""
        self.get_logger().debug(f'Loop running; planner_type = {self.planner_type_}, parameter = {self.get_parameter("planner_type").value}')

        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform... Have you launched SLAM?')
            return
        
        planner_str = self.get_parameter('planner_type').value
        
            # If currently inspecting an artifact
        if self.current_behavior_ == "inspection":
            # Wait until goal is reached or timeout is handled
            return

        if not self.ready_for_next_goal_:
            return

        # --- Progress flags ---
        if self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.get_logger().info('Reached first artifact!')
            self.reached_first_artifact_ = True
        if self.planner_type_ == PlannerType.RETURN_HOME:
            self.get_logger().info('Returned home!')
            self.returned_home_ = True
        

        if planner_str == 'frontier_exploration' and self.current_behavior_ == "exploration":
            self.planner_type_ = PlannerType.FRONTIER_EXPLORATION
        elif planner_str == 'random_goal':
            self.planner_type_ = PlannerType.RANDOM_GOAL

        #######################################################
        # Execute the planner by calling the relevant method
        # Add your own planners here!
        self.get_logger().info(f'Calling planner: {self.planner_type_.name}')

        if self.planner_type_ == PlannerType.FRONTIER_EXPLORATION:
            self.planner_frontier_exploration()
        elif self.planner_type_ == PlannerType.RANDOM_GOAL:
            self.planner_random_goal()
        elif self.planner_type_ == PlannerType.MOVE_FORWARDS:
            self.planner_move_forwards(10)
        elif self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.planner_go_to_first_artifact()
        elif self.planner_type_ == PlannerType.RETURN_HOME:
            self.planner_return_home()
        else:
            self.get_logger().error('No valid planner selected')
            self.destroy_node()


    def find_frontiers(self):
        """Find clusters of frontier cells and return centroids as candidate goals (only if enough unknown nearby)."""
 
        if not hasattr(self, 'map_data_'):
            self.get_logger().debug("No map data yet.")
            return []
    
        width = self.map_width_
        height = self.map_height_
        data = np.array(self.map_data_, dtype=np.int8).reshape((height, width))

        # Frontier mask: free cells (0) adjacent to unknown (-1)
        frontier_mask = np.zeros_like(data, dtype=bool)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:  # free cell
                    neighborhood = data[y-1:y+2, x-1:x+2]
                    if np.any(neighborhood == -1):
                        frontier_mask[y, x] = True

        # Label contiguous frontier clusters
        structure = np.ones((3, 3), dtype=int)
        labeled, num_features = label(frontier_mask, structure=structure)

        if num_features == 0:
            self.get_logger().debug("No frontier clusters found.")
            return []

        # Compute centroids (in world coordinates)
        centroids = []
        for i in range(1, num_features + 1):
            com = center_of_mass(frontier_mask, labels=labeled, index=i)
            if np.isnan(com[0]):
                continue

            # Convert to world coordinates
            wy = self.map_origin_.position.y + com[0] * self.map_resolution_
            wx = self.map_origin_.position.x + com[1] * self.map_resolution_

            # Check how many unknown cells surround this cluster
            fx = int((wx - self.map_origin_.position.x) / self.map_resolution_)
            fy = int((wy - self.map_origin_.position.y) / self.map_resolution_)
            r = int(1.0 / self.map_resolution_)  
            x_min, x_max = max(0, fx - r), min(width, fx + r)
            y_min, y_max = max(0, fy - r), min(height, fy + r)
            patch = data[y_min:y_max, x_min:x_max]
            unknown_count = np.sum(patch == -1)

            # Only accept frontiers with enough unexplored area
            if unknown_count >= self.min_unknown_cell_clusters:
                centroids.append((wx, wy))


        self.get_logger().info(f"Found {len(centroids)} valid frontier clusters (of {num_features} total).")
        return centroids



    def choose_frontier_goal(self, frontiers, robot_pose, min_dist=0.5):
        """Select the best frontier based on distance and information gain."""
        if not frontiers:
            return None

        best_score = float('-inf')
        best_frontier = None

        if self.ready_for_next_goal_:

            for f in frontiers:
                dist = math.hypot(f[0] - robot_pose.x, f[1] - robot_pose.y)
                if dist < min_dist:
                    continue

                score = -dist + 1.0 / (1.0 + math.exp(-0.2 * (dist - 1.0)))
                if score > best_score:
                    best_score = score
                    best_frontier = f

            if best_frontier:
                self.get_logger().info(
                    f"Chosen frontier: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}) with score={best_score:.2f}"
                )
            else:
                self.get_logger().warn("No valid frontier goal selected.")

        return best_frontier

    def planner_frontier_exploration(self):
        """Main frontier exploration loop."""

        # Skip if still travelling to a goal
        if not self.ready_for_next_goal_:
            return
        

        robot_pose = self.get_pose_2d()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot pose yet.")
            return

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().warn("No frontiers found. Maybe fully explored?")
            self.ready_for_next_goal_ = True
            return

        # Publish for RViz visualisation
        self.publish_frontier_markers(frontiers)

        # Choose next goal
        goal = self.choose_frontier_goal(frontiers, robot_pose)

        self.get_logger().info(f"Exploring frontier at ({goal[0]:.2f}, {goal[1]:.2f})")

        goal_pose = Pose2D(x=goal[0], y=goal[1], theta=0.0)
        self.current_goal_ = goal_pose
        self.planner_go_to_pose2d(goal_pose)




    def check_goal_timeout(self):
        """Check if the current goal has timed out."""
        inspection_timeout_placeholder = 25
        if self.goal_start_time_ is None or self.ready_for_next_goal_:
            return

        elapsed = (self.get_clock().now() - self.goal_start_time_).nanoseconds / 1e9

        # If we are inspecting, timeout should cancel inspection
        if self.current_behavior_ == "inspection":
            if elapsed > inspection_timeout_placeholder:
                self.get_logger().warn(f"Inspection goal timeout after {elapsed:.1f}s. Abandoning inspection.")
                self.handle_artifact_goal(success=False)
            return

        # Otherwise normal frontier/exploration timeout handling
        if self.current_behavior_ == "exploration":
            if elapsed > self.goal_timeout_sec_:
                self.get_logger().warn(f"Frontier goal timeout after {elapsed:.1f}s. Choosing new frontier.")
                self.ready_for_next_goal_ = True
                self.goal_start_time_ = None
                self.current_goal_ = None



    def publish_frontier_markers(self, frontiers):
        """Visualise detected frontier points in RViz as blue dots."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "frontiers"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.points = [Point(x=f[0], y=f[1], z=0.0) for f in frontiers]

        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.marker_pub_.publish(marker_array)



def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    while rclpy.ok():
        rclpy.spin(cave_explorer)