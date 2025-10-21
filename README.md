# Space Robotics Assignment 3

## Project
Project consists of three key areas:
- Perception
- Planning
- Advanced Tasks

Information regarding the development of these can be found in their respective markdown files:
- `perception.md`
- `planning.md`
- `advanced.md`

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

cd ros_workspace

colcon build --symlink-install --packages-select cave_explorer

source install/setup.bash
```

For convenience a bash script was created to do the above commands when a new terminal is created, please see `setup.sh`

## Launch Files

### Cave Explorer
The first launch file is `cave_explorer_startup.launch.py`, which launches the Gazebo simulator containing the robot and cave world, as well as the RVIZ visualisation window.
```sh
ros2 launch cave_explorer cave_explorer_startup.launch.py
```
NOTE: Due to some issues with WSL and Gazebo, normal launch may not work and hence you might need to use software rendering, so launch the startup file like this instead:
```sh
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch cave_explorer cave_explorer_startup.launch.py
```
### Cave Explorer Navigation
The second launch file is `cave_explorer_navigation.launch.py`, which launches some basic navigation capabilities including mapping and the Nav2 path planner pipeline for navigating around the environment.
```sh
ros2 launch cave_explorer cave_explorer_navigation.launch.py
``` 

### Cave Explorer Autonomous
The third and final launch file is `cave_explorer_autonomy.launch.py`, which launches the ROS node coded in `cave_explorer/cave_explorer.py`.
```sh
ros2 launch cave_explorer cave_explorer_autonomy.launch.py
```

### Camera Processor
This executes the new camera listener node that was created to take in data from the camera sensor.
```sh
ros2 run camera_processor listener
```

### Model Runner
This executes a node that handles the loading of a YOLO model (currently has to be configured in source code of this package).
```sh
ros2 run model_runner model_runner
```