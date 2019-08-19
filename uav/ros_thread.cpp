#include "ros_thread.h"
extern "C"{ 
#include <robotcontrol.h>
}
#include "uav.h"
#include "std_msgs/String.h"
#include <sstream>

static int ros_thread_ret_val;
static ros::NodeHandle nh_private("~");

void ros_setup(int* argc, char** argv) {
    ros::init(*argc, argv, "talker");
    std::string param;
    std::string check;
    nhPrivate.getParam("param", check);
    //std::cout << check << endl;
    if(check.compare("c") == 0){
        calibrate = true;
    } else if (check.compare("m") == 0){
        manual_mode = true;
    } else if (check.compare("h") == 0){
        manual_mode = true;
        printf("\nUSAGE: uav [-c|-m|-h]\n");
        printf("  -c  Calibrate and calculate offset for IMU.\n");
        printf("  -m  Maunal mode, will disable PID. Used to test motors. Should not be used for flight.\n");
        printf("  -h  Prints this help message.\n");
        exit(0);
    }
}

int ros_main()
{

    ros::Publisher chatter_pub = nhPrivate_ptr->advertise<std_msgs::String>("chatter", 1000);

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
    
    delete nhPrivate_ptr;
    return 0;
}



void* ros_thread_func() {
    ros_thread_ret_val = ros_main();
    if(ros_thread_ret_val == -1) {
        rc_set_state(EXITING);
    }

    return (void*)&ros_thread_ret_val;
}