#include "ros_thread.h"
extern "C"{ 
#include <robotcontrol.h>
}
#include <ros/ros.h>

static int ros_thread_ret_val;

int ros_main() {
    // // Announce this program to the ROS master as a "node" called "hello_world_node"
    // ros::init(argc, argv, "hello_world_node");
    // // Start the node resource managers (communication, time, etc)
    // ros::start(); // Broadcast a simple log message ROS_INFO_STREAM("Hello, world!");
    // while(rc_get_state() != EXITING) {

    //     // Process ROS callbacks until receiving a SIGINT (ctrl-c)
    //     ros::spin();
        
    // }
    // // Stop the node's resources
    // ros::shutdown(); 

     while(rc_get_state() != EXITING) {

        rc_usleep(2000000);
        
    }

    return 0;
}

void* ros_thread_func(void*) {
    ros_thread_ret_val = ros_main();
    if(ros_thread_ret_val == -1) {
        rc_set_state(EXITING);
    }

    return (void*)&ros_thread_ret_val;
}