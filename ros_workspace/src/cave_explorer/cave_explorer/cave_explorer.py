#!/usr/bin/env python3

import math
import random
from enum import Enum

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
    
    
    Planning1 = 10
    Planning2 = 11
    Planning3 = 12
    Advanced4 = 20


class CaveExplorer(Node):
    def __init__(self):
        super().__init__('cave_explorer_node')

        self.declare_parameter('mode', 'explorer') #THIS SHOULD HAVE ADVANCED 4, PLANNER 1, 2, 3 (OR WHATEVER WE CALL THEM)        

        #Planner 3 parameters
        self.inspected_artifacts_ = []
        self.preferable_artifacts_ = [1, 2, 3] #IDs of preferable artifacts to inspect first, could use names also (just dont know them all)
        self.current_artifacts_in_image_ = []
        self.inspect_attempts = 0 #number of attempts to inspect the current artifact. this can only hit 1 before we give up and move on
        self.inspect_time = 0 #how long we have spent inspecting the current artifact. if this gets too high we give up and move on

        # ===== Advanced 4: Online Roadmap Construction =====
        self.declare_parameter('roadmap_node_spacing', 0.4)   # meters between added nodes
        self.declare_parameter('roadmap_knn_k', 3)            # connect to K nearest neighbors
        self.declare_parameter('roadmap_edge_radius', 20.0)    # max distance to try edges
        self.declare_parameter('roadmap_occ_thresh', 20)      # occupancy threshold [0..100]

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

        # Remember the artifact locations
        # Array of type geometry_msgs.Point
        self.artifact_locations_ = []

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

        # Image processing
        self.cv_bridge_ = CvBridge()
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)            # Publish artefact detections to the visualiser thingy
        model_path = "/home/student/ros2_ws/src/space_robotics_a3/ros_workspace/src/model_runner/models/model_1/my_model.pt"                                  # NOTE: Relative to your current working directory
        self.model = YOLO(model_path)                                                               # Define YOLO model being used
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)  # Listen to camera sensor

        self.depth_image_sub_ = self.create_subscription(Image, 'camera/depth/image', self.depth_image_callback, 1)  # Listen to depth camera sensor chack this is the right topic

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.main_loop)
    
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

        # self.get_logger().warn('Map received:')
        # self.get_logger().warn(f'  xlim = [{self.xlim_[0]:.2f}, {self.xlim_[1]:.2f}]')
        # self.get_logger().warn(f'  ylim = [{self.ylim_[0]:.2f}, {self.ylim_[1]:.2f}]')
    
    def depth_image_callback(self, depth_image_msg):
        """
        Recieve a depth image.
        Use this method to help localise artifacts of interest.
        """
        # Turn received image into cv format
        depth_image = self.cv_bridge_.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
        
        # Debug
        self.get_logger().info('Depth image received from camera')
    
        pass
        # Process depth image here
        # Currently not implemented

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
            #number_of_boxes = len(boxes.xywh)   ####unsure if this is needed as the for loop wont run if theres no results right?
            #if number_of_boxes > 0:
                # Get name of detected object
            for box in boxes:
                class_id = int(box.cls)    ################## I think this si the artifact type (so for instance, 0 = backpack, 1 = mushroom, etc.)
                self.current_artifacts_in_image_.append(int(class_id)) #add the current artifact to the list of artifacts in the image
                self.get_logger().info('class_id: "%s"' % class_id)
                class_name = self.model.names[class_id] ##unsure what this is then these two varal are where im guessing i get the names from #
                self.get_logger().info('class_name: "%s"' % class_name)

            # Draw bounding boxes and labels
            for i in range(len(boxes.xywh)):
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

        # Set flags
        if self.artifact_found_:
            self.get_logger().info('Artifact found!')
            # TODO: Debug - Temporairly disable localisation
            # self.localise_artifact() 

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
        self.publish_artifact_markers() ##

    def publish_artifact_markers(self): ##as seen above it take the list of points and publishes them as markers
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



    ###FUNCTIONS HARLEYS ADDED TO MAKE PLANNER 3 WORK###
    def planner1(self):
        if self.planner_type_ == PlannerType.Planning3:
            ##add in autonomous searching for the artifacts
            if self.artifact_found_ == True:
                self.planner2()


    def planner2(self):
        
        if self.planner_type_ == PlannerType.Planning3:
            for artifact in self.current_artifacts_in_image_:
                if artifact in self.preferable_artifacts_:
                    


                    pass ## do the upclose search of it, must be able to timeout
            return ## return so it can continue autonomously searching in planner 1


    def planner3(self):
        """Add your own planner here!"""
        self.planner1()
        









        pass



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
    
    def _los_free(self, x1, y1, x2, y2):
        """
        Line-of-sight check between two world coordinates using Bresenham's algorithm on the occupancy grid.

        Returns True if every grid cell along the discrete line between (x1,y1) and (x2,y2) has occupancy <= roadmap_occ_thresh_.
        Returns False if the map is not available or any cell is considered occupied.
        """
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
        # need map + tf
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
        # publish if changed
        self._publish_roadmap()

    ###########################################################################################
    ############################### End of stuff for advaNCED 4########################


    def main_loop(self):
        # Always allow roadmap mode to run without Nav2 goals
        mode = self.get_parameter('mode').get_parameter_value().string_value.lower()

        # If SLAM/TF isn't up, do nothing
        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform... Have you launched a SLAM node?')
            return


        if mode == 'advanced4':
            self.roadmap_update()
            return



        if not self.ready_for_next_goal_:
            return

        # Progress flags for your template planners
        if self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.get_logger().info('Successfully reached first artifact!')
            self.reached_first_artifact_ = True
        if self.planner_type_ == PlannerType.RETURN_HOME:
            self.get_logger().info('Successfully returned home!')
            self.returned_home_ = True

        # Pick planner (your original logic)
        if not self.reached_first_artifact_:
            self.planner_type_ = PlannerType.GO_TO_FIRST_ARTIFACT
        elif not self.returned_home_:
            self.planner_type_ = PlannerType.RETURN_HOME
        else:
            self.planner_type_ = PlannerType.RANDOM_GOAL

        self.get_logger().info(f'Calling planner: {self.planner_type_.name}')

        if self.planner_type_ == PlannerType.MOVE_FORWARDS:
            self.ready_for_next_goal_ = False
            self.planner_move_forwards(10)
        elif self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.ready_for_next_goal_ = False
            self.planner_go_to_first_artifact()
        elif self.planner_type_ == PlannerType.RETURN_HOME:
            self.ready_for_next_goal_ = False
            self.planner_return_home()
        elif self.planner_type_ == PlannerType.RANDOM_WALK:
            self.ready_for_next_goal_ = False
            self.planner_random_walk()
        elif self.planner_type_ == PlannerType.RANDOM_GOAL:
            self.ready_for_next_goal_ = False
            self.planner_random_goal()
        else:
            self.get_logger().error('No valid planner selected')
            self.destroy_node()





def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    rclpy.spin(cave_explorer)
