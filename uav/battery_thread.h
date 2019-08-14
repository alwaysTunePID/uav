#ifndef BATTERY_THREAD_H
#define BATTERY_THREAD_H

typedef struct battery_info {
	double cell_voltage;
	double voltage;
	double battery_voltage;
	
}battery_info;

void* battery_thread_func(void*);

extern battery_info battery_data;
#endif