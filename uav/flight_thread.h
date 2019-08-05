#ifndef FLIGHT_THREAD_H
#define FLIGHT_THREAD_H

typedef struct esc_input_t{

	double u_1;
    double u_2;
    double u_3;
    double u_4;

} esc_input_t;

void* flight_thread_func();

#endif //FLIGHT_THREAD_H