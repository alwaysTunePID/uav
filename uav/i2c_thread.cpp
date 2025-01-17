/**
 * i2c_thread.cpp
 * 
 * Samples the i2c bus (IMU and barometer, if the barometer would have been used) 
 * This is because one need to keep functions which sample the i2c buss in the same thread.
*/
extern "C"
{ 
#include <robotcontrol.h>
}
#include "i2c_thread.h"
#include "imu.h"
#include "baro.h"
// Global variables for IMU/DMP
static int i2c_thread_ret_val;


int i2c_main(sem_t *IMU_sem){

    //if (initialize_baro()) return -1;
    if (initialize_imu() ) return -1;

    while(rc_get_state()!=EXITING){
	for (int i = 0; i < 10;i++)
	{
        	sample_imu(IMU_sem);
        	rc_usleep(100);
	}
	//sample_baro();
   
    }

    int r1 =finalize_imu();
    int r2 = 0; //finalize_baro();
    if (r1 || r2) return -1;


    return 0;
}


void* i2c_thread_func(void* IMU_sem) // wrapper function for the i2c main which casts the retval to void*
{
    i2c_thread_ret_val = i2c_main((sem_t*)IMU_sem);
    if (i2c_thread_ret_val)
    {
        rc_set_state(EXITING); // ism this needed? OBS
    }
    return (void*)&i2c_thread_ret_val;
}



