# EE346 Lab 4 Line_Follower & Pose estimation
Ensure that you have committed the commands below
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
$ roslaunch raspicam_node camerav2_410x308_30fps.launch
```
(at SBC)
```
cd /catkin_ws
roslaunch aruco_marker_finder.launch markerId:=14 markerSize:=0.05 
roslaunch line_follower_turtlebot lf1.launch
```
(at remote PC)
Warning! u should install the `aruco` pack first
```
sudo apt-get install ros-melodic-aruco-ros
```
