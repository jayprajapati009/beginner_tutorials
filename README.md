
  

# ROS2 Publisher and Subscriber

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

  

ROS2 beginner tutorial practice assignment for ENPM808X course at University of Maryland.

  

## Task

- Modify the publisher node to publish a custom string message

- Modify the tutorial code to follow Google C++ Style Guide (with course modifications)

- Run cpplint on your ROS package files and save the output as a text file to the results folder

- Run cppcheck on your ROS package files and save the output as a text file to the results folder

  

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
Now change the name of the directory to ```cpp_pubsub```
```sh
mv beginner_tutorials cpp_pubsub
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
colcon build --packages-select cpp_pubsub
```

  

## Run Instructions

After the successful build, to run open a new terminal,

```sh
cd ~/ros2_ws
```

```sh
. install/setup.bash
```

### Using the launch file

To run the launch file and initate the publisher, subscriber and the service,
```sh
cd ~/ros2_ws/src/cpp_pubsub/launch
```
```sh
ros2 launch week10_hw.yaml frequency:=20.0
```

### Launch the nodes
To launch the **publisher** node,
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```sh
ros2 run ros_pubsub talker
```
To launch the **subscriber** node,
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```sh
ros2 run ros_pubsub listener
```
To launch the **server** node,
```sh
cd ~/ros2_ws
```
```sh
. install/setup.bash
```
```sh
ros2 run ros_pubsub server
```

### Change the ```frequency``` parameter

```sh
ros2 param set \minimal_publisher freq 5.0
```

### To record a bag file

To run the launch file and initate the publisher, subscriber and the service,
```sh
cd ~/ros2_ws/src/cpp_pubsub/launch
```
```sh
ros2 launch cpp_pubsub bag_record_launch.py bag_record:=True
```

### To play a bag file

```sh
cd ~/ros2_ws/src/cpp_pubsub/results/bag_files
```
select the bag file you want to play

```sh
ros2 bag info <file_name>
```
then, start the listener node and,
```sh
ros2 bag play <file_name>
```
  

## Cppcheck and Cpplint

To run the Cpplint and the cppcheck command and save the results in the results directory,

```sh
sh cpplint_cppcheck.sh
```

## Result Screenshots

<img src="https://github.com/jayprajapati009/beginner_tutorials/blob/Week10_HW/results/terminal.png" alt="Terminal" width="600">

<img src="https://github.com/jayprajapati009/beginner_tutorials/blob/Week10_HW/results/rqt_log.png" alt="RQT Log" width="600">

<img src="https://github.com/jayprajapati009/beginner_tutorials/blob/Week10_HW/results/rqt_graph.png" alt="RQT Graph" width="600">

<img src="https://github.com/jayprajapati009/beginner_tutorials/blob/Week11_HW/results/launch_file_flag_util.png" alt="launch_file_flag_util" width="600">
<img src="https://github.com/jayprajapati009/beginner_tutorials/blob/Week11_HW/results/ros_bag_info.png" alt="ros_bag_info" width="600">


## References

[1] http://docs.ros.org/en/humble/index.html