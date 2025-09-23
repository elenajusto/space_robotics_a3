# ROS Primer
Reference notes to understand ROS architecture and components.

### Setting Up ROS
Need to source the bash file so that it setups up environment variables that allows ROS to function:
```sh
source /opt/ros/humble/setup.bash
```

To view the environement variables use the command:
```sh
printenv | grep -i ROS
```

Key environment variables to check are:
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

Set ROS to only communicate within `localhost`:
```sh
export ROS_LOCALHOST_ONLY=1
```

### Turtlesim
Turtlesim is a lightweight simulator to show what ROS can do.

Install with:
```sh
sudo apt install ros-humble-turtlesim
```

Check packages to confirm installation with:
```sh
ros2 pkg executables turtlesim
```

- `ros2`: Can use to start nodes, set paramaters, listen to topics.
- `rqt`: GUI tool that represents all the command line options of ROS.

Start turtlesim with:
```sh
ros2 run turtlesim turtlesim_node
```

Start a node to control the turtle:
```sh
ros2 run turtlesim turtle_teleop_key
```

### Viewing active topics, services and actions
```sh
ros2 nodelist

ros2 topic list

ros2 service list

ros2 action list
```

### Using `rqt`
Install `rqt` with:
```sh
sudo apt install '~nros-humble-rqt*'
```

Run `rqt` with:
```sh
rqt
```

From the menu select `Plugins > Services > Service Caller`

## Nodes
Each node in ROS should be responsible for a single, modular purpose.
Examples: 
- controlling the wheel motors
- publishing the sensor data 
![ros_node](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

## Topics

## Services

## Parameters

## Actions