# EE346 Lab 4 Line_Follower & Pose estimation
Ensure that you have committed the commands below
```
$ ssh pi@192.168.3.26
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
$ roslaunch raspicam_node camerav2_410x368.launch
```
(at SBC)
```
$ roscore
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
$ cd catkin_ws/src/EE346_lab6/
$ python navi.py
```
(at remote PC)

