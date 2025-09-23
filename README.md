# Space Robotics Assignment 3

## Project
High level overview is:
- Build a vision dataset of artifacts of interest for training a computer vision model 
- Detect and localise artifacts of interest as the robot moves through the cave 
- Autonomously explore, navigate and patrol the cave 
- Perform close-range inspection of discovered artifacts
- Construct a topological roadmap of the cave

Tasks are further broken down as follows:

### Perception 1: Collect an image dataset of the artifacts
- Goal: Create a dataset of artefacts of interest in the cave (rock thing, aliens, etc) as well as control images (wall, images with nothing).
- Can collect RGB images
- Can collect depth images

### Perception 2: Detect artifacts with computer vision
- Goal: Create a computer vision model to detect the above artefacts.
- Can use RGB camera and Depth camera

### Perception 3: Artifact localisation and display
- Goal: Estimate the location of the detected artefacts on the world map.
- Potential approaches:
    - Estimate direction from the pixel coordinates of the detection
    - Estimate distance using the depth camera
- Handle multiple detections

### Planning 1: Autonomously explore the cave
- Goal: Robot to explore cave and build map of new areas.
- aka reduce the number of unknown/unobserved grids/pixels on the map if that grid/pixel does not have an obstacle and is hence traversable.

### Planning 2: Close-range inspection
- Goal: Upon detection of an artefacte, pause exploration, generate a path to the artefact and navigate to it.

### Planning 3: Behaviour switching
- Goal: Alternate between exploration and inspection. Inspect new artefacts whilst not inspecting already inspected artefacts.

### Advanced 1: Robust perception
- Goal: Extend your solution and analysis from Perception 1-3 for environments with additional perceptual challenges. Apply degrading visual effects to the images received in cave_explorer.py, then feed these degraded images into your computer vision pipeline.
- Aim is to simulate Martian conditions. Examples include dust, poor lighting, low-quality cameras, dirty lenses, or motion blur.

### Advanced 2: Cave geometry analysis
- Goal: Extend your system to perform online analysis of cave geometry while exploring. Automatically identify regions of interest such as the widest open areas (potentially suitable for future human habitation) and narrow passages (critical for navigation and hazard assessment).

### Advanced 3: Communication network deployment
- Goal: Extend your system so that the robot maintains a multi-hop communication link back to the start location throughout its mission. The robot is able to deploy communication relay nodes in the cave, with the assumption that each node can connect to others via line-of-sight communication up to a fixed distance.

### Advanced 4: Online roadmap construction
- Goal: Build a navigation roadmap online as the robot explores the Martian cave.

### Advanced 5: Persistent monitoring
- Goal: Repeatedly visit a set of key points in the environment.

## Dependencies
```sh
sudo apt update  
sudo apt install ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo  
sudo apt install ros-humble-robot-localization  
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-xacro
```

## Set ROS Environment
```sh
source /opt/ros/humble/setup.bash
```

## Building
```sh
cd ros_workspace

colcon build --symlink-install --packages-select cave_explorer

source install/setup.bash
```

## Executing Launch Files
The first launch file is `cave_explorer_startup.launch.py`, which launches the Gazebo simulator containing the robot and cave world, as well as the RVIZ visualisation window.
```sh
ros2 launch cave_explorer cave_explorer_startup.launch.py
```

NOTE: Due to some issues with WSL and Gazebo, normal launch may not work and hence you might need to use software rendering, so launch the startup file like this instead:
```sh
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch cave_explorer cave_explorer_startup.launch.py
```

The second launch file is `cave_explorer_navigation.launch.py`, which launches some basic navigation capabilities including mapping and the Nav2 path planner pipeline for navigating around the environment.
```sh
ros2 launch cave_explorer cave_explorer_navigation.launch.py
``` 
    
The third and final launch file is `cave_explorer_autonomy.launch.py`, which launches the ROS node coded in `cave_explorer/cave_explorer.py`.
```sh
ros2 launch cave_explorer cave_explorer_autonomy.launch.py
```