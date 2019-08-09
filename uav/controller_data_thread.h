#ifndef CONTROLLER_DATA_THREAD_H
#define CONTROLLER_DATA_THREAD_H

typedef struct controller_data_t {
    double v_signals[4]; // v1, v2, v3, v4
    //esc_input_t esc_input_data; // u1, u2, u3, u4
    double angles[3]; //roll pitch yaw
    double rates[3]; //roll rate, pitch rate, yaw rate
    double rate_pid[3]; // (kp_r_r, kp_r_p, kp_r_y)*rate_error
    double k_angle_pid[2]; //(kp_a_r, kp_a_p)*angle_error
    double rate_refs[3]; // rate reference from angle PID
    double integral_pid[2];//I_a_r, I_a,p
    double rate_errors[3]; // roll, pitch, yaw error
    double angle_errors[2]; // roll, pitch

} controller_data_t;

controller_data_t controller_data;

void* controller_data_func();

#endif