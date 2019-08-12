#include "ros_thread.h"
#include <robotcontrol.h>

static int ros_thread_ret_val;

void* ros_thread_func() {
    while(rc_get_state() != EXITING) {
        //Do stuff
    }

    return (void*) ros_thread_ret_val;
}