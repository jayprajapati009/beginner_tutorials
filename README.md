
# ROS2 Publisher and Subscriber  
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ROS2 beginner tutorial practice assignment for ENPM808X course at University of Maryland.

## Task
-   Modify the publisher node to publish a custom string message
-   Modify the tutorial code to follow Google C++ Style Guide (with course modifications)
-   Run cpplint on your ROS package files and save the output as a text file to the results folder
-   Run cppcheck on your ROS package files and save the output as a text file to the results folder

## Dependencies
- rclcpp
- stdmsgs
- OS: Ubuntu Linux 20.04
- ROS Version: ROS2 Humble Hawksbill

## Build Instructions

Navigate to the source folder of the ROS2 workspace
```sh
cd ~/ros2_ws/src
```
Clone the GitHub repository
```sh
git clone https://github.com/jayprajapati009/beginner_tutorials.git
```
Now to build the package go to the root of the ROS2 workspace
```sh
cd ~/ros2_ws
```
check the dependencies
```sh  
rosdep install -i --from-path src --rosdistro humble -y
```
and build the package 
```sh
colcon build --packages-select beginner_tutorials
```

## Run Instructions
After the successful build, to run open a new terminal,
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```
ros2 run ros_pubsub talker
```
To run the subscriber node, open a new terminal
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```
ros2 run ros_pubsub listener
```

##  Cppcheck and Cpplint
To run the Cpplint and the cppcheck command and save the results in the results directory,
```sh
sh cpplint_cppcheck.sh
```