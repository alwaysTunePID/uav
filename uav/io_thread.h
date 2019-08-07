#ifndef IO_THREAD_H
#define IO_THREAD_H

void calibration_sleep();
void dsm_signal_loss_warning(uint64_t time_ns);
void dsm_signal_restored();
void* io_thread_func();

#endif