#ifndef DSM_THREAD_H
#define DSM_THREAD_H

#include <robotcontrol.h>

typedef struct dsm_entry_t {
	uint64_t time_ns;
	double channels[8];
} dsm_entry_t;

void *dsm_thread_func();
int log_dsm();
int get_latest_dsm(dsm_entry_t* d);


#endif //DSM_THREAD_H
