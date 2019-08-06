#include "io_thread.h"
#include "imu.h"
#include "battery_thread.h"
#include <stdio.h>
#include <robotcontrol.h>
#include <unistd.h>

#define GREEN "\033[1;32m"
#define YELLOW "\033[01;33m"
#define RED "\033[0;31m"
#define RESET_COLOR "\033[0m"

#define SPACE_BUFFER "                                                  "

static int io_thread_ret_val;

/* void set_K(inputs_t *p, int val) {
    pthread_mutex_lock(&(p->mutex));
    p->K = val;
    pthread_mutex_unlock(&(p->mutex));
} */

int io_main(void) {
	// double K;

    sleep(1);

    while (rc_get_state() != EXITING) {
		//Input
        //scanf(" %lf", &K);
        //printf("You passed the first scanf\n");
        //set_K(inputs, K);
        //fflush(stdin);
        //printf("K is %lf", K);

		//Output
        char color[8];

        if(battery_data.voltage > 11.8) color = GREEN;
        else if (battery_data.voltage > 10) color = YELLOW;
        else color = RED;

        printf("\r  Battery voltage: %s%lf%s V%s", color, battery_data.voltage, RESET_COLOR, SPACE_BUFFER);

		fflush(stdout);
		
        rc_usleep(500000);
    }

    return 0;
}

void calibration_sleep() {
    sleep(1);
    char loading[30] = "#.............................";
    for(int i = 1; i < 30 && rc_get_state() != EXITING; i++) {
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
