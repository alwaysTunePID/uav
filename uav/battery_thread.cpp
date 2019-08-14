#include <stdio.h>
#include "battery_thread.h"
#include "io_thread.h"
extern "C"
{ 
#include <robotcontrol.h>
}


battery_info battery_data;

void get_voltage() {
	battery_data.voltage = rc_adc_dc_jack();
	double cell = rc_adc_batt()/2.0;
	battery_data.cell_voltage = cell;
	battery_data.battery_voltage = cell*3.0;
	
	
}

int battery_main() {
	if(rc_adc_init()==1) return -1;
	while(rc_get_state()!=EXITING) {
		get_voltage();
		rc_usleep(5000000);	
	}
	rc_adc_cleanup();
	return 0;
	
}
static int battery_thread_ret_val;
void* battery_thread_func(void*) {
	battery_thread_ret_val = battery_main();
	if(battery_thread_ret_val) printio("battery_thread failed to initialize");
	return (void*)&battery_thread_ret_val;
}
