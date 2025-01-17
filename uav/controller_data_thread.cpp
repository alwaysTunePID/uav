/**
 * controller_data_thread.cpp
 * 
 * Thread that logs the key signals and values used by the controller. 
 * This is useful when trying to design a new controller or tune one.
 * 
*/
#include "controller_data_thread.h"
#include <stdlib.h>
#include <stdio.h>
extern "C"
{ 
#include <robotcontrol.h>
}

FILE* imu_data;
FILE* PID;
FILE* errors;
FILE* reference;
FILE* motors;

controller_data_t  controller_data;


static int init_file(const char* data, FILE** file){
    char filename[30];
    sprintf(filename,"controller_%s.log",data);
    *file = fopen(filename, "w");
    if(*file == NULL) {
        printf("Failed to open file");
        return -1;
    }
    return 0;
}

static int close_file(FILE** file) {
    if(*file == NULL) return -1;
    fclose(*file);
    return 0;
}

void print_data(const char* data, FILE** file) {
    if(!strcmp(data, "motor")){
        fprintf(*file, "%lf %lf %lf %lf\n",
        controller_data.v_signals[0],
        controller_data.v_signals[1],
        controller_data.v_signals[2],
        controller_data.v_signals[3]);
        return;
    }
    if(!strcmp(data, "imu_data")){
        fprintf(*file, "%lf %lf %lf %lf %lf\n",
        controller_data.angles[0],
        controller_data.angles[1],
        controller_data.rates[0],
        controller_data.rates[1],
        controller_data.rates[2]);
        return;
    }
    if(!strcmp(data, "errors")) {
        fprintf(*file, "%lf %lf %lf %lf %lf\n",
        controller_data.angle_errors[0],
        controller_data.angle_errors[1],
        controller_data.rate_errors[0],
        controller_data.rate_errors[1],
        controller_data.rate_errors[2]);
        return;
    }

    if(!strcmp(data, "PID")) {
        fprintf(*file, "%lf %lf %lf %lf %lf %lf %lf\n",
        controller_data.k_angle_pid[0],
        controller_data.k_angle_pid[1],
        controller_data.rate_pid[0],
        controller_data.rate_pid[1],
        controller_data.rate_pid[2],
        controller_data.integral_pid[0],
        controller_data.integral_pid[1]);
        return;
    }

    if(!strcmp(data, "reference")) {
        fprintf(*file, "%lf %lf %lf\n",
        controller_data.rate_refs[0],
        controller_data.rate_refs[1],
        controller_data.rate_refs[2]);
        return;
    }


}

int controller_data_main() {
    //motor, imu_data, errors, PID, reference
    // can init more files if more data is needed
    // create more file pointers for more data
    if(init_file("imu_data",&imu_data)) return -1;
    //return init_file("errors",&errors);

    while(rc_get_state()!=EXITING){
        print_data("imu_data",&imu_data);
        //print_data("errors",&controller_data,&errors);
        rc_usleep(100000);

    }
    close_file(&imu_data);
    //close_file(&errors);
    return 0;
}

static int controller_data_ret_val;
void* controller_data_func(void*){
    controller_data_ret_val=controller_data_main();
    if(controller_data_ret_val) printf("controller_data failed to initialize\n");
	return (void*)&controller_data_ret_val;
}
