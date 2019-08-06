#ifndef BATTERY_THREAD_H
#define BATTERY_THREAD_H
#endif
typedef struct battery_info {
	double cell_voltage;
	double voltage;
	double battery_voltage;
	
}battery_info;

void* battery_thread_func();

battery_info battery_data;
