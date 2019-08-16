#include "ros_thread.h"
extern "C"{ 
#include <robotcontrol.h>
}
#include "ros/ros.h"
#include "uav.h"
#include "std_msgs/String.h"
#include <sstream>

static int ros_thread_ret_val;

int ros_main(arg_holder_t* arg_struct)
{
    ros::init(*(arg_struct->argc_ptr), arg_struct->argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (rc_get_state() != EXITING && ros::ok()){

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }

    return 0;
}



void* ros_thread_func(void* arg_struct) {
    ros_thread_ret_val = ros_main((arg_holder_t*) arg_struct);
    if(ros_thread_ret_val == -1) {
        rc_set_state(EXITING);
    }

    return (void*)&ros_thread_ret_val;
}