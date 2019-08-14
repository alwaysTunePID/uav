#include "ros_thread.h"
extern "C"
{ 
#include <robotcontrol.h>
}

static int ros_thread_ret_val;

int ros_main() {
    while(rc_get_state() !=EXITING) {
        rc_usleep(100000000);
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