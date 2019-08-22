#!/bin/bash
source devel/setup.bash
sudo chown root:root ~/catkin_ws/devel/lib/uav_pkg/uav
sudo chmod u+s ~/catkin_ws/devel/lib/uav_pkg/uav
rosrun uav_pkg uav _param:=c

