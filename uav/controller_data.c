#include "controller_data.h"
#include <stdlib.h>
#include <stdio.h>
#include <robotcontrol.h>

FILE* file;


static int init_file(char* type){
    char*filename;
    sprintf(filename,"controller_%s.log",type);
    file = fopen(filename, 'w');
    if(file == NULL) {
        printf("Failed to open file");
        return -1;
    }
    return 0;
}

static int close_file() {
    if(file == NULL) return -1;
    fclose(file);
}

void print_data(char* data, controller_data_t* controller_data) {
    if(!strcmp(data, "motor")){
        fprintf(file, "%lf %lf %lf %lf",
        controller_data->v_signals[0],
        controller_data->v_signals[1],
        controller_data->v_signals[2],
        controller_data->v_signals[3]);
        return;
    }
    if(!strcmp(data, "imu_data")){
        fprintf(file, "%lf %lf %lf %lf %lf",
        controller_data->angles[0],
        controller_data->angles[1],
        controller_data->rates[0],
        controller_data->rates[1],
        controller_data->rates[2]);
        return;
    }
    if(!strcmp(data, "errors")) {
        fprintf(file, "%lf %lf %lf %lf %lf",
        controller_data->angle_errors[0],
        controller_data->angle_errors[1],
        controller_data->rate_errors[0],
        controller_data->rate_errors[1],
        controller_data->rate_errors[2]);
        return;
    }

    if(!strcmp(data, "PID")) {
        fprintf(file, "%lf %lf %lf %lf %lf %lf %lf",
        controller_data->k_angle_pid[0],
        controller_data->k_angle_pid[1],
        controller_data->rate_pid[0],
        controller_data->rate_pid[1],
        controller_data->rate_pid[2],
        controller_data->integral_pid[0],
        controller_data->integral_pid[1]);
        return;
    }

    if(!strcmp(data, "reference")) {
        fprintf(file, "%lf %lf %lf",
        controller_data->rate_refs[0],
        controller_data->rate_refs[1],
        controller_data->rate_refs[2]);
        return;
    }


}

int controller_data_main() {
    if(init_file("PID")) return -1;
    char* data_selection="PID";
    while(rc_get_state()!=EXITING){
        print_data(data_selection,&controller_data);
        rc_usleep(100000);

    }
    close_file();
    return 0;
}

static int controller_data_ret_val;
void* controller_data_func(){
    controller_data_ret_val=controller_data_main();
    if(controller_data_return_val) printf("controller_data failed to initialize\n");
	return (void*)&controller_data_ret_val;
}