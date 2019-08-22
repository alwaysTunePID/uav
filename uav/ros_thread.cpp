#include "ros_thread.h"
extern "C"{ 
#include <robotcontrol.h>
}
#include "uav.h"
#include "std_msgs/String.h"
#include <sstream>
#include <queue>
#include <vector>
#include <semaphore.h>

#define MAX_MESSAGE_LENGTH 255

enum RosPublishChannel {
    chatter,
    errors,
    position
};

enum RosListenChannel {
    misc,
    pos
};

static int ros_thread_ret_val;

/*
        +--------------+
        | ROS-Varibles |
        +--------------+
*/


// //Publishers
// static ros::Publisher chatter_pub = nh_private.advertise<std_msgs::String>("chatter", 1000);
// static ros::Publisher errors_pub = nh_private.advertise<std_msgs::String>("errors", 1000);
// static ros::Publisher position_pub = nh_private.advertise<std_msgs::String>("position", 1000);//TODO implement custom type

// //Subscribers
// static ros::Subscriber misc_sub = nh_private.subscribe("misc", 1000, listen_misc);
// static ros::Subscriber pos_sub = nh_private.subscribe("pos", 1000, listen_pos);

//Queues for listen-function
static std::queue<int> chatter_q;

//Varibles for listen-latest-function
static std::string pos_v;

ros::NodeHandle* ros_setup(int* argc, char** argv) {
    ros::init(*argc, argv, "droneA");//TODO pass node name as commandline argument
    ros::NodeHandle* nh_private_ptr = new ros::NodeHandle;
    std::string check;
    nh_private_ptr->getParam("param", check);
    
    if(check.compare("c") == 0){
        calibrate = true;
        std::cout << "c was detected" << std::endl;
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
    return nh_private_ptr;
}

// void ros_publish(RosPublishChannel ros_channel, string fmt, ...) 
//     char message[MESSAGE_MAX_LENGTH];
//     char* output = new char[MESSAGE_MAX_LENGTH];
//     va_list args;
//     va_start(args, fmt);
//     vsprintf(message, fmt, args);
//     va_end(args);

//     switch(ros_channel) {
//     case chatter:
//     case errors:
//         std_msgs::String msg;
//         std::stringstream ss;
//         ss << message;
//         msg.data = ss.str();
//         break;
//     case position:
//         break;
//     }

//     switch(ros_channel) {
//     case chatter:
//         chatter_pub.publish(msg);
//         break;
//     }

//     ros::spinOnce();
// }

// void ros_listen(RosPublishChannel ros_channel) {

// }

// void ros_listen_latest() {

// }

// void listen_misc(const std_msgs::String::ConstPtr& msg) {
//     misc_q.push(msg->data.c_str());
// }

// void listen_pos(const std_msgs::String::ConstPtr& msg) {
//     pos_v = msg->data.c_str();
// }

int ros_main(ros::NodeHandle* nh_private_ptr) {//DEPRECATED
    // return 0;
    ros::Publisher chatter_pub = nh_private_ptr->advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (rc_get_state() != EXITING && ros::ok()) {
    //     //Publisher
    //     while(!messages.empty()) {
    //         std_msgs::String msg;

    //         std::stringstream ss;
    //         ss << messages.front().second << count;
    //         msg.data = ss.str();

    //         switch(mesages.front().first) {
    //         case chatter:
    //             chatter_pub.publish(msg);
    //             break;
    //         }

    //         ros::spinOnce();
            
    //         delete messages.front().first;
    //         delete messages.front().second;
    //         messages.pop()
    //     }

        //Listener
        //Listen, add to corrent queue(one per channel?), post to semaphore
        loop_rate.sleep();
    }
    delete nh_private_ptr; 
    
    return 0;
}



void* ros_thread_func(void* nh_private_ptr) {
    ros_thread_ret_val = ros_main((ros::NodeHandle*)nh_private_ptr);
    if(ros_thread_ret_val == -1) {
        rc_set_state(EXITING);
    }

    return (void*)&ros_thread_ret_val;
}