#!/bin/bash
# It became a hassle just to run a ros program

export ROS_HOSTNAME=192.168.7.2
export ROS_MASTER_URI=http://192.168.7.2:11311
export ROS_IP=192.168.7.2
roscore &
sleep 13s

source devel/setup.bash
chown root:root ~/catkin_ws/devel/lib/uav_pkg/uav
chmod u+s ~/catkin_ws/devel/lib/uav_pkg/uav
rosrun uav_pkg uav _param:=c

