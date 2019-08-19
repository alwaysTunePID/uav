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

enum RosChannel {
    chatter
}

static int ros_thread_ret_val;
static ros::NodeHandle nh_private("~");
static std::queue<std::pair<RosChannel, std::string>> pub_messages;

sem_t ROS_sem;

void ros_setup(int* argc, char** argv) {
    sem_init(&ROS_sem);

    ros::init(*argc, argv, "talker");
    std::string check;
    nhPrivate.getParam("param", check);
    
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

void ros_publish(RosChannel ros_channel, string message) {
    RosChannel* rc = new RosChannel;
    std::string* msg = new std::string;

    *rc = ros_channel;
    *msg = message;

    messages.push({*rc, *msg});
}

void ros_listen(RosChannel ros_channel) {

}

int ros_main() {
    ros::Publisher chatter_pub = nhPrivate_ptr->advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (rc_get_state() != EXITING && ros::ok()) {
        //Publisher
        while(!messages.empty()) {
            std_msgs::String msg;

            std::stringstream ss;
            ss << messages.front().second << count;
            msg.data = ss.str();

            switch(mesages.front().first) {
            case chatter:
                chatter_pub.publish(msg);
                break;
            }

            ros::spinOnce();
            
            delete messages.front().first;
            delete messages.front().second;
            messages.pop()
        }

        //Listener
        //Listen, add to corrent queue(one per channel?), post to semaphore
        loop_rate.sleep();
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