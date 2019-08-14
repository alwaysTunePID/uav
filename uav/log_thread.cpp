#include "log_thread.h"
extern "C"
{ 
#include <robotcontrol.h>
}
#include <semaphore.h>
#include "imu.h"
#include "baro.h"
#include "dsm_thread.h"
#include "gps.h"
#include "airspeed.h"

#define LOG_DELAY_US   2000000

static int log_thread_ret_val;


void* log_thread_func(void*)
{
	rc_usleep(LOG_DELAY_US);
	//int counter = 0;
	while(rc_get_state()!=EXITING){
		 
		log_imu();
		log_dsm();
		rc_usleep(5000);
    	//  log_gps();
    	//  log_airspeed();
		// Log baro in 25 Hz
		// if (counter == 8){
		// 	log_baro();
		// 	counter=0;
		// }
		// counter++; 
	}
	log_thread_ret_val = 0;
	return (void*)&log_thread_ret_val;
}
