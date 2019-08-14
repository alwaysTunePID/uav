#ifndef IO_THREAD_H
#define IO_THREAD_H
#include <stdio.h>
#include <stdint.h>

void calibration_sleep();
void dsm_signal_loss_warning(uint64_t time_ns);
void dsm_signal_restored();
void set_armed(int in_armed);
void update_value(double a);
void set_warning(const char* fmt, ...);
void resolve_warning();
void set_error(const char* fmt, ...);
void resolve_error();
void printio(const char* fmt, ...);
void* io_thread_func(void*);
void update_value(double value);

#endif