#ifndef IO_THREAD_H
#define IO_THREAD_H
#include <stdio.h>
#include <stdint.h>

void calibration_sleep();
void dsm_signal_loss_warning(uint64_t time_ns);
void dsm_signal_restored();
void set_armed(int in_armed);
void update_value(double a);
void printio(const char* fmt, ...);
void* io_thread_func();
void update_value(double value);

#endif