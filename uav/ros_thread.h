#ifndef ROS_THREAD_H
#define ROS_THREAD_H
#include "ros/ros.h"

void* ros_thread_func(void*);
ros::NodeHandle* ros_setup(int*, char**);

#endif