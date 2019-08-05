#include "input_thread.h"
#include "uav.h"
#include <stdio.h>
#include <robotcontrol.h>
#include <baro.h>



static int input_thread_ret_val;

/* void set_K(inputs_t *p, int val) {
    pthread_mutex_lock(&(p->mutex));
    p->K = val;
    pthread_mutex_unlock(&(p->mutex));
} */

int input_main(inputs_t *inputs) {
	double K;
	bmp_entry_t bmp_data;

    while (rc_get_state() != EXITING) {
		//Input
        //scanf(" %lf", &K);
        //printf("You passed the first scanf\n");
        //set_K(inputs, K);
        //fflush(stdin);
        //printf("K is %lf", K);

		//Output
		int baro = get_latest_baro(&bmp_data);
		fprintf("\rBarometer data: %i", baro);
		fflush();
		
        rc_usleep(500000);
    }

    return 0;
}


void *input_thread_func(void *arg) {
    input_thread_ret_val = input_main((inputs_t*)arg);
    if (input_thread_ret_val ==-1) {
        rc_set_state(EXITING);
    }
	
    return (void*)&input_thread_ret_val;
}
