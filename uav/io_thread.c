#include "io_thread.h"
#include "imu.h"
#include <stdio.h>
#include <robotcontrol.h>
#include <unistd.h>


static int io_thread_ret_val;

/* void set_K(inputs_t *p, int val) {
    pthread_mutex_lock(&(p->mutex));
    p->K = val;
    pthread_mutex_unlock(&(p->mutex));
} */

int io_main(void) {
	// double K;

    while (rc_get_state() != EXITING) {
		//Input
        //scanf(" %lf", &K);
        //printf("You passed the first scanf\n");
        //set_K(inputs, K);
        //fflush(stdin);
        //printf("K is %lf", K);

		//Output
		//printf("\rp_angle_error: %lf r_angle_error %lf", p_angle_error, r_angle_error);
		//fflush(stdout);
		
        rc_usleep(500000);
    }

    return 0;
}

void update_values(double p, double r) {
    printf("\rp_angle_error: %lf r_angle_error %lf", p, r);
	fflush(stdout);
}

void calibration_sleep() {
    sleep(1);
    char loading[30] = "#.............................";
    for(int i = 1; i < 30; i++) {
        loading[i] = '#';
        printf("\r  Calibrating IMU [%s] %d s / 30 s", loading, (i+1));
        fflush(stdout);
        sleep(1);
    }

    printf("\r  Calibrating IMU [%s] Done!       \n", loading);
    fflush(stdout);
}


void *io_thread_func(void) {
    io_thread_ret_val = io_main();
    if (io_thread_ret_val ==-1) {
        rc_set_state(EXITING);
    }
	
    return (void*)&io_thread_ret_val;
}
