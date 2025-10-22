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

    FRONTIER_EXPLORATION = 6

    Advanced4 = 20


class CaveExplorer(Node):
    def __init__(self):
        super().__init__('cave_explorer_node')

        self.declare_parameter('mode', 'advanced4') #THIS SHOULD HAVE ADVANCED 4, PLANNER 1, 2, 3 (OR WHATEVER WE CALL THEM)
        # ===== Advanced 4: Online Roadmap Construction =====
        self.declare_parameter('roadmap_node_spacing', 0.4)   # meters between added nodes
        self.declare_parameter('roadmap_knn_k', 3)            # connect to K nearest neighbors
        self.declare_parameter('roadmap_edge_radius', 5)    # max distance to try edges
        self.declare_parameter('roadmap_occ_thresh', 50)      # occupancy threshold [0..100]

        self.roadmap_node_spacing_ = float(self.get_parameter('roadmap_node_spacing').value)
        self.roadmap_knn_k_        = int(self.get_parameter('roadmap_knn_k').value)
        self.roadmap_edge_radius_  = float(self.get_parameter('roadmap_edge_radius').value)
        self.roadmap_occ_thresh_   = int(self.get_parameter('roadmap_occ_thresh').value)

        # storage for advanced 4
        self.road_nodes_ = []      # [(x,y)]
        self.road_edges_ = []      # [(i,j)]
        self._last_node_idx_ = None
        self._roadmap_dirty_ = False

        # markers used in the advanced 4 stuff
        self.road_nodes_marker_ = Marker()
        self.road_nodes_marker_.header.frame_id = "map"
        self.road_nodes_marker_.ns   = "roadmap"
        self.road_nodes_marker_.id   = 100
        self.road_nodes_marker_.type = Marker.POINTS
        self.road_nodes_marker_.action = Marker.ADD
        self.road_nodes_marker_.scale.x = 0.25
        self.road_nodes_marker_.scale.y = 0.25
        self.road_nodes_marker_.color.a = 1.0
        self.road_nodes_marker_.color.r = 1.0
        self.road_nodes_marker_.color.g = 1.0
        self.road_nodes_marker_.color.b = 0.0

        self.road_edges_marker_ = Marker()
        self.road_edges_marker_.header.frame_id = "map"
        self.road_edges_marker_.ns   = "roadmap"
        self.road_edges_marker_.id   = 101
        self.road_edges_marker_.type = Marker.LINE_LIST
        self.road_edges_marker_.action = Marker.ADD
        self.road_edges_marker_.scale.x = 0.08
        self.road_edges_marker_.color.a = 1.0
        self.road_edges_marker_.color.r = 0.2
        self.road_edges_marker_.color.g = 0.6
        self.road_edges_marker_.color.b = 1.0

        # Publisher for roadmap visualization
        self.roadmap_pub_ = self.create_publisher(MarkerArray, 'marker_array_roadmap', 10)

        self.seed_grid_spacing_ = 1.0  # meters between grid
        self.seed_per_cycle_ = 5 # max nodes to add per cycles

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

        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker_array_artifacts', 10) # Creat publisher for marker array, but new marker is created for each location
        self.marker_artifacts_array = []

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

        self.cv_bridge_ = CvBridge()
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)            # Publish artefact detections to the visualiser thingy
        model_path = "/home/eleanorlow/spcrob/team_project/space_robotics_a3/ros_workspace/src/model_runner/models/model_1/my_model.pt"                                  # NOTE: Relative to your current working directory
        self.model = YOLO(model_path)                                                               # Define YOLO model being used
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)  # Listen to camera sensor
        self.current_image_id = 0

        # # Prepare image processing and subscribe to image detection to get artifact information
        # self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
        # self.declare_parameter('computer_vision_model_filename', rclpy.Parameter.Type.STRING)
        # self.computer_vision_model_ = cv2.CascadeClassifier(self.get_parameter('computer_vision_model_filename').value)
        # self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)

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
        self.inspection_duplicate_distance_ = 4.0  # don't inspect same artifact if within this distance

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
        self.map_origin_ = map_msg.info.origin
        self.map_resolution_ = map_msg.info.resolution
        self.map_height_ = map_msg.info.height
        self.map_width_ = map_msg.info.width
        self.map_data_ = map_msg.data

        # Set current limits
        self.xlim_ = [self.map_origin_.position.x, self.map_origin_.position.x + self.map_width_ * self.map_resolution_]
        self.ylim_ = [self.map_origin_.position.y, self.map_origin_.position.y + self.map_height_ * self.map_resolution_]


    def image_callback(self, image_msg):
        """
        Recieve an RGB image.
        Use this method to detect artifacts of interest.
        
        Code integrated based on dev and testing done in the `model_runner` package
        """
        self.current_artifacts_in_image_ = [] #reset the list of artifacts in the image

        # Turn received image into cv format
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
        # Debug
        # self.get_logger().info('Image received from camera')
    
        # Execute computer vision model
        results = self.model(image, stream=True, conf=0.5)  # Configure to detect when confidence > 50%

        if (results):
            self.artifact_found_ = True

        # Process detection
        for result in results:
            # Boxes object for bounding box outputs
            boxes = result.boxes  

            # Process any detections if they exist
            #number_of_boxes = len(boxes.xywh)   ####unsure if this is needed as the for loop wont run if theres no results right?
            #if number_of_boxes > 0:
                # Get name of detected object
            for box in boxes:
                class_id = int(box.cls)    ################## I think this si the artifact type (so for instance, 0 = backpack, 1 = mushroom, etc.)
                self.current_artifacts_in_image_.append(int(class_id)) #add the current artifact to the list of artifacts in the image
                # self.get_logger().info('class_id: "%s"' % class_id)
                class_name = self.model.names[class_id] ##unsure what this is then these two varal are where im guessing i get the names from #
                # self.get_logger().info('class_name: "%s"' % class_name)
                self.current_image_id = class_id

            # Draw bounding boxes and labels
            for i in range(len(boxes.xywh)):
                self.get_logger().info('box: "%s"' % boxes.xywh[i])

                x = int(boxes.xywh[i][0])
                y = int(boxes.xywh[i][1])
                width = int(boxes.xywh[i][2])
                height = int(boxes.xywh[i][3])

                # self.get_logger().info('x: "%s"' % x)
                # self.get_logger().info('y: "%s"' % y)
                # self.get_logger().info('width: "%s"' % width)
                # self.get_logger().info('height: "%s"' % height)

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

        #If an artifact is found, switch to inspection mode
        if self.artifact_found_ and self.current_behavior_ != "inspection":
            self.localise_artifact()
    


    def plan_inspection_goal(self):
        """Generate and send a close-range (standoff) goal near the selected artifact."""

        artifact_point = self.selected_artifact_

        if not self.selected_artifact_:
            self.get_logger().warn("plan_inspection_goal: no selected artifact Switching back to exploration.")
            self.current_behavior_ = "exploration"
            return

        
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
        self.publish_artifact_markers()
        # Mark artifact inspected (avoid duplicates)
        if self.selected_artifact_ is not None:
            self.inspected_artifacts_.append(self.selected_artifact_)

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
        point.x = robot_pose.x + 5
        point.y = robot_pose.y + 5
        # point.x = 18.1 # Forcing a fake location for artifacts
        # point.y = 6.6 
        # point.z = 1.0
        

        skip_artifact = False

        # check duplicates to not inspect artifatcs already inspected
        for a in self.inspected_artifacts_:
            if math.hypot(a.x - point.x, a.y - point.y) < self.inspection_duplicate_distance_:
            # if point == a:
                self.get_logger().info("Detected artifact already inspected. Skipping artifact.")  
                skip_artifact = True
                return

        #Will add to locations
        for a in self.artifact_locations_:
            if math.hypot(a.x - point.x, a.y - point.y) < self.inspection_duplicate_distance_:
            # if point == a:
                self.get_logger().info("Artifact already recorded. Skipping artifact.")
                skip_artifact = True
                return

        # Save approx artifact location and publish markers
        # self.publish_artifact_markers()
        

        # select this artifact to inspect and plan approach
        self.selected_artifact_ = point

        if not skip_artifact:
            #Setting behavious as inspection as artifact should be inspected
            self.artifact_locations_.append(point)
            self.current_behavior_ = "inspection"
            self.get_logger().info("New artifact detect, switching to inspection mode.")
            self.plan_inspection_goal()

    def publish_artifact_markers(self):
        """Publish all detected artifact markers with colors by class type."""

        # Creating new marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "artifacts"
        marker.id = len(self.marker_artifacts_array)
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.orientation.w = 1.0

        # Assigning colour depending on artifact type
        color_map = {
            0: (1.0, 0.0, 0.0),  # red
            1: (0.0, 1.0, 0.0),  # green
            2: (0.0, 0.0, 1.0),  # blue
            3: (1.0, 1.0, 0.0),  # yellow
            4: (1.0, 0.0, 1.0),  # magenta
            5: (0.0, 1.0, 1.0),  # cyan
        }

        r, g, b = color_map.get(self.current_image_id, (1.0, 1.0, 1.0))
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # Add artifact position
        marker.points = [self.selected_artifact_]

        # Add to the array and publish
        self.marker_artifacts_array.append(marker)

        marker_array_msg = MarkerArray()
        marker_array_msg.markers = self.marker_artifacts_array
        self.marker_pub_.publish(marker_array_msg)
        self.get_logger().info(f"Published artifact marker (id={marker.id}, color=({r:.1f},{g:.1f},{b:.1f}))")



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
            # don't set ready_for_next_goal_ True yet
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
        mode = self.get_parameter('mode').get_parameter_value().string_value.lower()
        planner_str = self.get_parameter('planner_type').value

        self.get_logger().debug(f'Loop running; planner_type = {self.planner_type_}, parameter = {self.get_parameter("planner_type").value}')

        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform... Have you launched SLAM?')
            return

        if mode == 'advanced4':
            self.roadmap_update()
        
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

    #####################################################################################################
    #################### ADVANCED 4 FUNCTIONS TO BUILD ONLINE ROADMAP ##################################
    #####################################################################################################

    def _grid_at(self, ix, iy): #check if the position is in the map
        if ix < 0 or iy < 0 or ix >= self.map_width_ or iy >= self.map_height_:
            return 100  # out of map = blocked
        return self.map_data_[iy*self.map_width_ + ix]  ##why is this calculation done like this?
       # This is done to convert the 2D grid coordinates (ix, iy) into a 1D array index.
       # The map_data_ array is stored in row-major order, so we need to calculate the index
       # by multiplying the row index (iy) by the width of the map (map_width_) and adding the column index (ix).

    def _world_to_grid(self, x, y): 
        ix = int((x - self.map_origin_.position.x) / self.map_resolution_)
        iy = int((y - self.map_origin_.position.y) / self.map_resolution_)
        return ix, iy

    def _grid_to_world(self, ix, iy):
        x = self.map_origin_.position.x + (ix + 0.5) * self.map_resolution_ #the 0.5 is to get the center of the cell
        y = self.map_origin_.position.y + (iy + 0.5) * self.map_resolution_
        return x, y
    
    #used this to find the nearest existing node to the xy position
    def _nearest_node_dist(self, x, y):
        if not self.road_nodes_:
            return float('inf'), None
        best_d = float('inf')
        best_idx = None
        for i, (nx, ny) in enumerate(self.road_nodes_):
            d = math.hypot(x - nx, y - ny)
            if d < best_d:
                best_d, best_idx = d, i
        return best_d, best_idx

    def _seed_unreached_free_areas(self):
        """Add roadmap nodes in observed but unreached free cells."""
        if not hasattr(self, 'map_data_') or self.map_resolution_ <= 0:
            return

        # Convert spacing from meters to grid cells
        step_cells = max(1, int(self.seed_grid_spacing_ / self.map_resolution_))
        added = 0

        if not self.road_nodes_:
            return  # Wait until we have a node from robot motion

        for iy in range(0, self.map_height_, step_cells):
            for ix in range(0, self.map_width_, step_cells):
                if added >= self.seed_per_cycle_:
                    return

                occ = self._grid_at(ix, iy)
                if occ != 0:  # only perfectly free cells
                    continue

                xw, yw = self._grid_to_world(ix, iy)
                d, nn_idx = self._nearest_node_dist(xw, yw)
                if d < self.roadmap_node_spacing_:
                    continue

                # check LOS to nearest node
                if nn_idx is None:
                    continue
                nx, ny = self.road_nodes_[nn_idx]
                if not self._los_free(xw, yw, nx, ny):
                    continue

                # add node and connect
                self.road_nodes_.append((xw, yw))
                new_idx = len(self.road_nodes_) - 1
                self._try_connect_edges_from(new_idx)
                self._roadmap_dirty_ = True
                added += 1

        if added > 0:
            self._publish_roadmap()


    
    def _los_free(self, x1, y1, x2, y2):
        #Check the line-of-sight betwee the two wold coordinatesusing the beewssmans algorithm on the occupancy gird.
        #it should retun true if every gird cell along the discrete line between (x1,y1) and (x2,y2) has an occupancy that is less than the set threahold (whihc is currently 50)
        #will return a big fat false is the map has something blocking the LOS, so there is something in the way and the threshold isnt met.
        #also if the mapdata isnt there.

        # Ensure we have a map to work with
        if not hasattr(self, 'map_data_'):
            return False
        # Convert world coordinates to integer grid indices
        ix1, iy1 = self._world_to_grid(x1, y1)
        ix2, iy2 = self._world_to_grid(x2, y2)

        # Bresmans algorythm gotten from online. Need all the comments to remmebr whats happening
        #https://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python

        # dx is the absolute difference in x indices
        dx = abs(ix2 - ix1)
        # dy is the negative absolute difference in y indices (algorithm convention)
        dy = -abs(iy2 - iy1)
        # Step direction for x and y (either +1 or -1)
        sx = 1 if ix1 < ix2 else -1
        sy = 1 if iy1 < iy2 else -1
        # The error term used by Bresenham
        err = dx + dy

        # Start from the first grid cell
        x, y = ix1, iy1

        # Walk the grid cells from (ix1,iy1) to (ix2,iy2)
        while True:
            # If the cell occupancy is above threshold, line-of-sight is blocked
            if self._grid_at(x, y) > self.roadmap_occ_thresh_:
                return False

            # If we've reached the target cell, the line is free
            if x == ix2 and y == iy2:
                break

            # Double the error to decide which way to step
            e2 = 2 * err
            # Step in x if warranted
            if e2 >= dy:
                err += dy
                x += sx
            # Step in y if warranted
            if e2 <= dx:
                err += dx
                y += sy

        # No blocking cells found along the line
        return True

    def _maybe_add_node(self, pose: Pose2D): # Add a node at the robot pose if spaced far enough from last node.
        if len(self.road_nodes_) == 0:
            self.road_nodes_.append((pose.x, pose.y))
            self._last_node_idx_ = 0
            self._roadmap_dirty_ = True
            return

        lastx, lasty = self.road_nodes_[self._last_node_idx_]
        d = math.hypot(pose.x - lastx, pose.y - lasty)
        if d >= self.roadmap_node_spacing_:
            self.road_nodes_.append((pose.x, pose.y))
            self._last_node_idx_ = len(self.road_nodes_) - 1
            self._roadmap_dirty_ = True

    def _try_connect_edges_from(self, idx):
        """KNN connections with LOS check."""
        if idx is None: return
        x, y = self.road_nodes_[idx]
        # candidates within radius
        dists = []
        for j, (xj, yj) in enumerate(self.road_nodes_):
            if j == idx: continue
            dist = math.hypot(x - xj, y - yj)
            if dist <= self.roadmap_edge_radius_:
                dists.append((dist, j))
        dists.sort(key=lambda t: t[0])
        added = 0
        for _, j in dists:
            if added >= self.roadmap_knn_k_:
                break
            # avoid duplicate edge
            if (idx, j) in self.road_edges_ or (j, idx) in self.road_edges_:
                continue
            xj, yj = self.road_nodes_[j]
            if self._los_free(x, y, xj, yj):
                self.road_edges_.append((idx, j))
                added += 1
                self._roadmap_dirty_ = True

    def _publish_roadmap(self):
        self.get_logger().warn('Publishing roadmap...')
        if not self._roadmap_dirty_:
            return
        # nodes
        self.get_logger().info(f"Publishing roadmap: {len(self.road_nodes_)} nodes, {len(self.road_edges_)} edges")

        self.road_nodes_marker_.points = []
        for (x,y) in self.road_nodes_:
            p = Point(); p.x = x; p.y = y; p.z = 0.0
            self.road_nodes_marker_.points.append(p)

        # edges (LINE_LIST expects pairs of points)
        self.road_edges_marker_.points = []
        for (i,j) in self.road_edges_:
            p1 = Point(); p1.x, p1.y, p1.z = self.road_nodes_[i][0], self.road_nodes_[i][1], 0.0
            p2 = Point(); p2.x, p2.y, p2.z = self.road_nodes_[j][0], self.road_nodes_[j][1], 0.0
            self.road_edges_marker_.points.extend([p1,p2])

        arr = MarkerArray()
        # update headers
        now = self.get_clock().now().to_msg()
        self.road_nodes_marker_.header.stamp = now
        self.road_edges_marker_.header.stamp = now
        arr.markers = [self.road_nodes_marker_, self.road_edges_marker_]
        self.roadmap_pub_.publish(arr)
        self._roadmap_dirty_ = False

        self.get_logger().info("Roadmap Markers Published.")


    def roadmap_update(self): #this should be called a few time a second when the planner type is correct

        if not hasattr(self, 'map_data_'):
            return
        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            return
        pose = self.get_pose_2d()
        if pose is None:
            return

        # add node if spaced
        prev_count = len(self.road_nodes_)
        self._maybe_add_node(pose)
        if len(self.road_nodes_) != prev_count:
            # new node → try KNN edges from the new node
            self._try_connect_edges_from(self._last_node_idx_)

        self._publish_roadmap()
        self._seed_unreached_free_areas()




def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    while rclpy.ok():
        rclpy.spin(cave_explorer)