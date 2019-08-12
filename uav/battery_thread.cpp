#include <stdio.h>
#include <robotcontrol.h>
#include "battery_thread.h"
#include "io_thread.h"


void get_voltage(battery_info * battery_levels) {
	battery_levels->voltage = rc_adc_dc_jack();
	double cell = rc_adc_batt()/2.0;
	battery_levels->cell_voltage = cell;
	battery_levels->battery_voltage = cell*3.0;
	
	
}

int battery_main() {
	if(rc_adc_init()==1) return -1;
	while(rc_get_state()!=EXITING) {
		get_voltage(&battery_data);
		rc_usleep(5000000);	
	}
	rc_adc_cleanup();
	return 0;
	
}
static int battery_thread_ret_val;
void* battery_thread_func() {
	battery_thread_ret_val = battery_main();
	if(battery_thread_ret_val) printio("battery_thread failed to initialize");
	return (void*)&battery_thread_ret_val;
}
