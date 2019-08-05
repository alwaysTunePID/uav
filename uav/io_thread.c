#include "io_thread.h"
#include "uav.h"
#include "imu.h"
#include <stdio.h>
#include <robotcontrol.h>



static int input_thread_ret_val;

/* void set_K(inputs_t *p, int val) {
    pthread_mutex_lock(&(p->mutex));
    p->K = val;
    pthread_mutex_unlock(&(p->mutex));
} */

int io_main(void) {
	// double K;
	imu_entry_t imu_data;

    while (rc_get_state() != EXITING) {
		//Input
        //scanf(" %lf", &K);
        //printf("You passed the first scanf\n");
        //set_K(inputs, K);
        //fflush(stdin);
        //printf("K is %lf", K);

		//Output
		get_latest_imu(&imu_data);
		printf("\rTemp: %lf", imu_data.temp);
		fflush(stdout);
		
        rc_usleep(500000);
    }

    return 0;
}


void *io_thread_func(void) {
    input_thread_ret_val = io_main();
    if (input_thread_ret_val ==-1) {
        rc_set_state(EXITING);
    }
	
    return (void*)&input_thread_ret_val;
}
