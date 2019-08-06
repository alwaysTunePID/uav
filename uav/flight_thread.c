#include <stdio.h>
#include <unistd.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <semaphore.h>
#include "imu.h"
#include "baro.h"
#include "dsm_thread.h"
#include "flight_thread.h"
#include "circular_buffer.h"
#include "io_thread.h"
#include "uav.h"

/*
This thread is intended to handle the flying of the drone. All control signals related to the flying will be calculated here. 
The ESC control signals will be set from here. Input from the controller will be passed through here.


OBS. 
IF dsm data is read from the log file, then the drone is impossible to control since of a 5 sec time delay.
Reasons for this?
Could be "to many uses of get_latest_dsm"? 


*/
#define MIN_ERROR 0.3
#define MAX_I 0.4
#define FREQUENCY 200.0
#define TS 1/FREQUENCY
#define MAX_ROLL_ANGLE 20.0
#define MAX_PITCH_ANGLE 20.0
#define MAX_YAW_RATE 90.0 


// IMU Data
static imu_entry_t imu_data;
static esc_input_t esc_input;
// static bmp_entry_t baro_data;


// misc variables
static int flight_thread_ret_val;						 
static int armed = 0;				 // Static mabye?
static int pid_active = 0;			 // Switch to 1 if you want to use the PID
static int const_alt_active = 0;	// Switch to 1 if you want to keep constant altitude. UNUSED
static uint64_t dsm_nanos;			 // nanoseconds since last dsm packet


static FILE* controller_log;

// Control signals
static double v_p;
static double v_r;
static double v_y;

// Storage variables for angles
static double pitch = 0.0;
static double roll = 0.0;
static double yaw = 0.0;

// altitude controller reference signal. UNUSED
//double alt_ref = 0.0;

// Normalized signals to ecs

static double v_1;
static double v_2;
static double v_3;
static double v_4;

// Used for updating PID parameters. UNUSED
// double input_K = 1.0;
// double input_D = 1.0;
// double input_Y = 1.0;
// int help = 1;



// ----------------------------------------------------------------------------------------
// ------------------------------------------ new pid design ------------------------------
// ----------------------------------------------------------------------------------------

// Errors

static double p_rate = 0.0;
static double r_rate = 0.0;
static double y_rate = 0.0;

static double pitch_rate_ref = 0.0;
static double roll_rate_ref = 0.0;
static double yaw_rate_ref = 0.0;


static double p_rate_error = 0.0;
static double r_rate_error = 0.0;
static double y_rate_error = 0.0;


// rate pid
static double K_pr_p = 0.0014; //0.001 
static double K_pr_r = 0.0014; //0.001 could be higher
static double K_pr_y = 0.0046;
// angle pid
static double K_pa_p = 2.0;
static double K_pa_r = 2.0;
//static double K_pa_y = 3.0;

static double K_ia_p = 0.002*5.0;
static double K_ia_r = 0.002*5.0;
//static double K_ia_y = 0.002*4.0;

static double I_a_p = 0.0;
static double I_a_r = 0.0;
//static double I_a_y = 0.0;


// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

// Function for saturating values
static double clip(double n, double min, double max){

	if (n < min)
		return min;
	if (n > max)
		return max;
	return n;
}

static double abs_fnc(double n){
	if (n < 0.0)
		return n * -1.0;
	return n;
}


// Implemented for changing parameters through keyboard. Unused
// int get_K(inputs_t *p)
// {
//     int K;
//     pthread_mutex_lock(&(p->mutex));
//     K = p->K;
//     pthread_mutex_unlock(&(p->mutex));

//     return K;
// }

static int init_controller_log_file(){

	controller_log = fopen("controller.log", "w");

	if (controller_log == NULL){
		printf("Could not open a controller log file!\n");
		return -1;
	}
	//fprintf(controller_log," temp_c alt_m pressure_pa\n");
	return 0;
}

static int close_controller_log(){
	// add flag for enable and disable logging?
	if (controller_log == NULL){
		return -1;
	}
	fclose(controller_log);
	return 0;
}

// log file for controller data. 
int log_controller(esc_input_t *esc_input){

	if (controller_log == NULL){
		printf("Tried to write before controller log file was created\n");
		return -1;
	}

	fprintf(controller_log, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			esc_input->u_1,
			v_1,
			esc_input->u_2,
			v_2,
			esc_input->u_3,
			v_3,
			esc_input->u_4,
			v_4,
			pitch,
			roll,
			p_rate,
			r_rate,
			I_a_p,
			I_a_r,
			v_p,
			v_r);
	// fprintf(controller_log, "%f %f\n", pitch, roll);
	return 0;
}

void calibrate_IMU(sem_t* IMU_sem, double* mean_pitch_offset, double* mean_roll_offset) {
	calibration_sleep();

	sem_wait(IMU_sem);
	get_latest_imu(&imu_data);
	pitch = imu_data.euler[0] * RAD_TO_DEG;
	roll = imu_data.euler[1] * RAD_TO_DEG;

	*mean_pitch_offset = pitch;
	*mean_roll_offset = roll;

	for(int mean_samples = 1; mean_samples < 100; mean_samples++) {
		sem_wait(IMU_sem);
		get_latest_imu(&imu_data);
		pitch = imu_data.euler[0] * RAD_TO_DEG;
		roll = imu_data.euler[1] * RAD_TO_DEG;

		*mean_pitch_offset = (double)(mean_samples - 1) / ((double)mean_samples) * *mean_pitch_offset + pitch / ((double)mean_samples);
		*mean_roll_offset = (double)(mean_samples - 1) / ((double)mean_samples) * *mean_roll_offset + roll / ((double)mean_samples);
	}

	printf("Pitch offset: ");
	printf("%8.4f ", *mean_pitch_offset);
	printf("\n");

	printf("Rolll offset: ");
	printf("%8.4f ", *mean_roll_offset);
	printf("\n");

	FILE* calibration_file = fopen("pitch_roll_offest.cal", "w");

	if (calibration_file == NULL){
		fprintf(stderr, "Could not open a calibration file!\n");
	}

	fprintf(calibration_file, "%lf %lf", *mean_pitch_offset, *mean_roll_offset);
	
	fclose(calibration_file);
}

void load_offset(double* mean_pitch_offset, double* mean_roll_offset) {
	FILE* calibration_file = fopen("pitch_roll_offest.cal", "r");

	if (calibration_file == NULL){
		fprintf(stderr, "Could not open a calibration file! Use flag -c if you have not created one.\n");
	}

	fscanf(calibration_file, "%lf %lf", mean_pitch_offset, mean_roll_offset);
	
	fclose(calibration_file);

	printf("Pitch offset: ");
	printf("%8.4f ", *mean_pitch_offset);
	printf("\n");

	printf("Rolll offset: ");
	printf("%8.4f ", *mean_roll_offset);
	printf("\n");
}

// Used to update parameters live through the controller. UNUSED. NEEDS UPDATE
/*static void update_PID_param(void){
	double scale_k = 0.001;
	double scale_d = 0.01;

	input_K = rc_dsm_ch_normalized(8);
	input_D = rc_dsm_ch_normalized(9);
	//input_Y = rc_dsm_ch_normalized(9);
	if (input_K * scale_k + K_p_0 < 0)
		help = 0;
	if (input_D * scale_d + T_d_p_0 < 0)
		help = 0;

	// if (input_Y * 0.01 + k_y_0 < 0)
	// 	help = 0;
	if (rc_dsm_ch_normalized(7) > 0.5){
		if (help){
			K_p = K_p_0 + input_K * scale_k;
			K_r = K_r_0 + input_K * scale_k;
			T_d_p = T_d_p_0 + input_D * scale_d;
			T_d_r = T_d_r_0 + input_D * scale_d;
			//K_y = k_y_0 + input_Y * 0.001;
			printf("New K_p is set %lf\n", K_p);
			printf("New K_r is set %lf\n", K_r);
			printf("New T_d_p is set %lf\n", T_d_p);
			printf("New T_d_r is set %lf\n\n", T_d_r);
			printf("New K_y is set %lf\n", K_y);
		}
		help = 0;
	}
	if (rc_dsm_ch_normalized(7) < 0.5){
		help = 1;
	}
}
*/

// Manual mode that runs if the PID is turned off
void manual_output(esc_input_t *esc_input, double thr){
	thr = (thr > 1.0) ? 1.0 : thr;
	thr = (thr < 0.05) ? -0.1 : thr;
 
	esc_input->u_1 = thr;
	esc_input->u_2 = thr;
	esc_input->u_3 = thr;
	esc_input->u_4 = thr;
}


// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------- Cascaded controllers--------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------

void rate_PID(esc_input_t *esc_input, double thr){
	p_rate_error = pitch_rate_ref - p_rate;
	r_rate_error = roll_rate_ref - r_rate;
	y_rate_error = yaw_rate_ref - y_rate;

	v_p = K_pr_p * p_rate_error;
	v_r = K_pr_r * r_rate_error;
	v_y = K_pr_y * y_rate_error;

	// Calulate new signal for each ESC
	// Motor 4 is top right, then goes clockwise bottom right is 1.
	v_1 = thr + v_p + v_r + v_y;
	v_2 = thr + v_p - v_r - v_y;
	v_3 = thr - v_p - v_r + v_y;
	v_4 = thr - v_p + v_r - v_y;

	// Make sure the control signals isnt out of range
	esc_input->u_1 = (v_1 > 1.0) ? 1.0 : v_1;
	esc_input->u_2 = (v_2 > 1.0) ? 1.0 : v_2;
	esc_input->u_3 = (v_3 > 1.0) ? 1.0 : v_3;
	esc_input->u_4 = (v_4 > 1.0) ? 1.0 : v_4;

	esc_input->u_1 = (v_1 < -0.1) ? -0.1 : v_1;
	esc_input->u_2 = (v_2 < -0.1) ? -0.1 : v_2;
	esc_input->u_3 = (v_3 < -0.1) ? -0.1 : v_3;
	esc_input->u_4 = (v_4 < -0.1) ? -0.1 : v_4;

	// Log controller data
	log_controller(esc_input);

}

void angle_PID(double* pitch_ref, double* roll_ref, double* yaw_ref){
	double p_angle_error = *pitch_ref - pitch;
	double r_angle_error = *roll_ref - roll;
	I_a_p = I_a_p + K_ia_p * TS * p_angle_error;
	I_a_r = I_a_r + K_ia_r * TS * r_angle_error;
	//I_a_y = I_a_y + K_ia_y * TS * y_angle_error;
	
	I_a_p = (abs_fnc(p_angle_error) < MIN_ERROR) ? 0.0 : I_a_p;
	I_a_r = (abs_fnc(r_angle_error) < MIN_ERROR) ? 0.0 : I_a_r;
	//I_a_y = (abs_fnc(y_angle_error) < MIN_ERROR) ? 0.0 : I_a_y;

	I_a_p = clip(I_a_p, -MAX_I, MAX_I);
	I_a_r = clip(I_a_r, -MAX_I, MAX_I);
	//I_a_y = clip(I_a_y, -MAX_I, MAX_I);

	pitch_rate_ref = K_pa_p * p_angle_error + I_a_p;
 	roll_rate_ref = K_pa_r * r_angle_error + I_a_r;
	yaw_rate_ref = *yaw_ref;

}

// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
int flight_main(sem_t *IMU_sem){
	init_controller_log_file();
	
	double mean_roll_offset = 0;
	double mean_pitch_offset = 0;
	
	double pitch_ref = 0;
	double roll_ref = 0;
	double yaw_ref = 0;
	double thr = 0; // normalized throttle

	//Calibrate
	if(calibrate) calibrate_IMU(IMU_sem, &mean_pitch_offset, &mean_roll_offset);
	else load_offset(&mean_pitch_offset, &mean_roll_offset);

	while (rc_get_state() != EXITING){
		dsm_nanos = rc_dsm_nanos_since_last_packet();

		
		if (rc_dsm_ch_normalized(5) > 0.7){
			armed = 1;
			rc_led_set(RC_LED_RED, 1);
		} else {
			armed = 0;
			rc_led_set(RC_LED_RED, 0);
		}

		if (rc_dsm_ch_normalized(6) > 0.7){
			pid_active = 1;
			rc_led_set(RC_LED_GREEN, 1);
		} else {
			pid_active = 0;
			rc_led_set(RC_LED_GREEN, 0);
		}
		// Height regulator. UNUSED
		if (rc_dsm_ch_normalized(7) > 0.7){ 
			// if (const_alt_active == 0)
			// {
			// 	get_latest_baro(&baro_data);
			// 	alt_ref = baro_data.bmp_data.alt_m;
			// 	printf("Baro ref alt: ");
			// 	printf("%8.2f ", baro_data.bmp_data.alt_m);
			// 	printf("\n");
			// 	const_alt_active = 1;
			// }
		} else {
			const_alt_active = 0;
		}
		// For this to work the controller output MUST be calibrated so that the signals always stay in the interval [-1,1]
		// Since d.channels[1-7] gives a double [-1,1] this will be interpreted as an angel, this must be scaled to
		// allow for a reasonable change in control signal.

		// multiply dsm signal with max angle or rate
		roll_ref = -rc_dsm_ch_normalized(2) * MAX_ROLL_ANGLE;
		pitch_ref = rc_dsm_ch_normalized(3) * MAX_PITCH_ANGLE;
		yaw_ref = -rc_dsm_ch_normalized(4) * MAX_YAW_RATE;
		
		sem_wait(IMU_sem);
		get_latest_imu(&imu_data);

		pitch = imu_data.euler[0] * RAD_TO_DEG - mean_pitch_offset;
		roll = imu_data.euler[1] * RAD_TO_DEG - mean_roll_offset;
		yaw = imu_data.gyro[2];

		p_rate = imu_data.gyro[0];
		r_rate = imu_data.gyro[1];
		y_rate = imu_data.gyro[2];

		if (dsm_nanos > 200000000){
			rc_servo_send_esc_pulse_normalized(0, 0);
			printf("\rSeconds since last DSM packet: %.2f              ", dsm_nanos / 1000000000.0);
		}
		else {

			// How to get data from log threads;
			
			thr = rc_dsm_ch_normalized(1);
			// bound the signal to the escs
			if (thr < -0.1)
				thr = -0.1;
			if (thr > 1.0)
				thr = 1.0;

			if (pid_active == 1){
				
				// update_PID_param();
				// PID_controller(TS, &esc_input, thr);
				angle_PID(&pitch_ref, &roll_ref, &yaw_ref);
				rate_PID(&esc_input,thr);
			
			}
			else {
				manual_output(&esc_input, thr);
			}
			if (armed == 1){
				rc_servo_send_esc_pulse_normalized(1, esc_input.u_1);
				rc_servo_send_esc_pulse_normalized(3, esc_input.u_2);
				rc_servo_send_esc_pulse_normalized(5, esc_input.u_3);
				rc_servo_send_esc_pulse_normalized(7, esc_input.u_4);
			}
			else {
				rc_servo_send_esc_pulse_normalized(0, 0);
			}
		}

		// sleep roughly enough to maintain FREQUENCY
		rc_usleep(1000000 / FREQUENCY);
	}
	close_controller_log();

	return 0;
}

void *flight_thread_func(sem_t *IMU_sem){
	flight_thread_ret_val = flight_main(IMU_sem);
	if (flight_thread_ret_val == -1){
		rc_set_state(EXITING);
	}
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 0);
	return (void *)&flight_thread_ret_val;
}