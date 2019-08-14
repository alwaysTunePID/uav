#include <stdio.h>
#include <unistd.h>
extern "C"
{ 
#include <robotcontrol.h>
} // includes ALL Robot Control subsystems
#include <semaphore.h>
#include "gps.h"
#include "i2c_thread.h"
#include "dsm_thread.h"
#include "log_thread.h"
#include "telemetry.h"
#include "airspeed.h"
#include "flight_thread.h"
#include "io_thread.h"
#include "battery_thread.h"
#include "controller_data_thread.h"
#include "ros_thread.h"
// threads  

static pthread_t i2c_thread;
static pthread_t dsm_thread;
//static pthread_t gps_thread;
static pthread_t log_thread;
static pthread_t telemetry_thread;
//static pthread_t airspeed_thread;  
static pthread_t flight_thread;
static pthread_t io_thread;
static pthread_t battery_thread;
static pthread_t controller_data_thread;
//static pthread_t test_thread;
static pthread_t ros_thread;

int calibrate = 0;
int manual_mode = 0;

/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
    if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
    else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
    return;
}

/**
 * If the user holds the pause button for 2 seconds, set state to EXITING which
 * triggers the rest of the program to exit cleanly.
 **/
void on_pause_press()
{
    int i;
    const int samples = 100; // check for release 100 times in this period
    const int us_wait = 2000000; // 2 seconds

    // now keep checking to see if the button is still held down
    for(i=0;i<samples;i++){
        rc_usleep(us_wait/samples);
        if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
    }
    printf("Long press detected, shutting down\n");
    rc_set_state(EXITING);
    return;
}

int main(int argc, char *argv[])
{
    int c;
    sem_t IMU_sem;

    while((c = getopt(argc, argv, "cmh")) != -1) {
        switch(c) {
        case 'c':
            calibrate = 1;
            break;
        case 'm':
            manual_mode = 1;
            break;
        case 'h':
            printf("\nUSAGE: uav [-c|-m|-h]\n");
            printf("  -c  Calibrate and calculate offset for IMU.\n");
            printf("  -m  Maunal mode, will disable PID. Used to test motors. Should not be used for flight.\n");
            printf("  -h  Prints this help message.\n");
            return 0;
        }
    }
    
    
    int ret;
    void* thread_retval; // return value of the thread
    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

    // start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: Failed to start signal handler\n");
        return -1;
    }

    sem_init(&IMU_sem, 0, 0);

    // initialize pause button
    if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
                      RC_BTN_DEBOUNCE_DEFAULT_US)){
        fprintf(stderr,"ERROR: Failed to initialize pause button\n");
        return -1;
    }
    // Assign functions to be called when button events occur
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);

    //start threads

	
     if(rc_pthread_create(&i2c_thread, i2c_thread_func, (void*)&IMU_sem, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start I2C sampler thread\n");
        return -1;
    }
	
    if(rc_pthread_create(&dsm_thread, dsm_thread_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start DSM thread\n");
        return -1;
    }
	
    /*
    if(rc_pthread_create(&gps_thread, gps_thread_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start GPS thread\n");
        return -1;
    }
	*/
    if(rc_pthread_create(&battery_thread, battery_thread_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start Battery monitor thread\n");
        return -1;
    }
    
    if(rc_pthread_create(&log_thread, log_thread_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start LOG thread\n");
        return -1;
    }

    if(rc_pthread_create(&telemetry_thread, telemetry_thread_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start LOG thread\n");
        return -1;
    }
    
	/*
    if(rc_pthread_create(&airspeed_thread, airspeed_thread_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start AIRSPEED thread\n");
        return -1;
    }
	*/
	
	if (rc_pthread_create(&flight_thread, flight_thread_func, (void*)&IMU_sem , SCHED_OTHER, 0)) {
		fprintf(stderr, "ERROR: Failed to start Flight thread\n");
		return -1;
	}
	 
	if (rc_pthread_create(&io_thread, io_thread_func, NULL, SCHED_OTHER, 0)) {
		fprintf(stderr, "ERROR: Failed to start input/output thread\n");
		return -1;
	}
	if(rc_pthread_create(&battery_thread, battery_thread_func, NULL, SCHED_OTHER, 0)) {
		fprintf(stderr, "ERROR: Failed to start input/output thread \n");
	}

    if(rc_pthread_create(&controller_data_thread, controller_data_func, NULL, SCHED_OTHER, 0)){
        fprintf(stderr, "ERROR: Failed to start input/output thread \n");
    }

    if(rc_pthread_create(&ros_thread, ros_thread_func, NULL, SCHED_OTHER, 0)) {
        fprintf(stderr, "ERROR: Failed to start ROS thread \n");
    }

    // Sleep and let threads work
    while(rc_get_state()==RUNNING){
        rc_usleep(10000);
    }
	
    //join i2c thread
    ret = rc_pthread_timed_join(i2c_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: IMU thread timed out\n");
    }
    printf("I2c thread returned:%d\n",*(int*)thread_retval);
	
    // dsm i2c thread
    ret = rc_pthread_timed_join(dsm_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: DSM thread timed out\n");
    }
    printf("DSM thread returned:%d\n",*(int*)thread_retval);

    // join gps thread
	/*
    ret = rc_pthread_timed_join(gps_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: GPS thread timed out\n");
    }
    printf("GPS thread returned:%d\n",*(int*)thread_retval);
	*/
    // join battery thread
    ret = rc_pthread_timed_join(battery_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: Battery monitor thread timed out\n");
    }
    printf("Battery monitor thread returned:%d\n",*(int*)thread_retval);
    
    // join log thread
    ret = rc_pthread_timed_join(log_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: LOG thread timed out\n");
    }
    printf("LOG thread returned:%d\n",*(int*)thread_retval);

    // join telemetry thread
    ret = rc_pthread_timed_join(telemetry_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: LOG thread timed out\n");
    }
    printf("TELEMETRY thread returned:%d\n",*(int*)thread_retval);
	/*
    // join airspeed thread
    ret = rc_pthread_timed_join(airspeed_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: AIRSPEED thread timed out\n");
    }
    printf("AIRSPEEL thread returned:%d\n",*(int*)thread_retval);
	*/


	// // join Flight thread
	ret = rc_pthread_timed_join(flight_thread, &thread_retval, 1.5);
	if (ret == 1) {
		fprintf(stderr, "ERROR: Flight thread timed out\n");
	}
	printf("Flight thread returned:%d\n", *(int*)thread_retval);

	ret = rc_pthread_timed_join(io_thread, &thread_retval, 1.5);
	if (ret == 1) {
	    fprintf(stderr, "ERROR: Input_thread timed out\n");
	}
    printf("IO thread returned %d\n", *(int*)thread_retval);
	

    ret = rc_pthread_timed_join(ros_thread, &thread_retval, 1.5);
    if(ret == 1) {
        fprintf(stderr, "ERROR: ROS thread timed out\n");
    }
    printf("ROS thread returned:%d\n", *(int*)thread_retval);

    ret = rc_pthread_timed_join(controller_data_thread, &thread_retval, 1.5);
    if ( ret == 1){
        fprintf(stderr,"ERROR: controller_data_thread timed out\n");
    }
    printf("controller_data_thread returned:%d\n",*(int*)thread_retval);

	

    // turn off LEDs and close file descriptors
    rc_led_set(RC_LED_GREEN, 0);
    rc_led_set(RC_LED_RED, 0);
    rc_led_cleanup();
    rc_button_cleanup();	// stop button handlers
    rc_remove_pid_file();	// remove pid file LAST
    sem_destroy(&IMU_sem);


    return 0;
}


