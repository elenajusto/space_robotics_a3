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
Each node in ROS should be responsible for a single, modular purpose. Nodes can send and receive data from other nodes via `topics`, `services`, `actions`, or `parameters`. A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

Examples: 
- controlling the wheel motors
- publishing the sensor data 
![ros_node](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

### Turtleism example
Start the main node:
```sh
ros2 run turtlesim turtlesim_node
```
See the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them:
```sh
ros2 node list
```
Start another node:
```sh
ros2 run turtlesim turtle_teleop_key
```
Access information ( list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node.) on a node:
```sh
ros2 node info <node_name>
```

## Topics

## Services

## Parameters

## Actions