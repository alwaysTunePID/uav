#include <stdio.h>
#include <unistd.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <semaphore.h>
#include "imu.h"
#include "baro.h"
#include "battery_thread.h"
#include "dsm_thread.h"
#include "flight_thread.h"
#include "circular_buffer.h"
#include "io_thread.h"
#include "uav.h"
#include "controller_data_thread.h"

/*
This thread is intended to handle the flying of the drone. All control signals related to the flying will be calculated here. 
The ESC control signals will be set from here. Input from the controller will be passed through here.


OBS. 
IF dsm data is read from the log file, then the drone is impossible to control since of a 5 sec time delay.
Reasons for this?
Could be "to many uses of get_latest_dsm"? 


*/
#define DESCEND_THR 0.58// With 0.6 it landed gracefully (but bonced on the floor)
#define MIN_ERROR 0.3
#define MAX_I 50
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
static int const_alt_active = 0;	// Switch to 1 if you want to keep constant altitude. UNUSED



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
static double K_pa_p = 4.0;
static double K_pa_r = 4.0;
//static double K_pa_y = 3.0;

static double K_ia_p = 0.002*5000.0; // Increase these
static double K_ia_r = 0.002*5000.0; // Increase these
//static double K_ia_y = 0.002*4.0;

static double I_a_p = 0.0;
static double I_a_r = 0.0;
//static double I_a_y = 0.0;


static double P_a_p = 0.0;
static double P_a_r = 0.0;


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

static int lost_dsm_connection(){
	return rc_dsm_nanos_since_last_packet() > 1000000000;
}




void LP_filter(double input, double* average, double factor){
	*average = input*factor + (1-factor)* *average;  // ensure factor belongs to  [0,1]
}

void calibrate_IMU(sem_t* IMU_sem, double* mean_pitch_offset, double* mean_roll_offset, double* mean_g) {
	calibration_sleep();

	sem_wait(IMU_sem);
	get_latest_imu(&imu_data);
	pitch = imu_data.euler[0] * RAD_TO_DEG;
	roll = imu_data.euler[1] * RAD_TO_DEG;

	*mean_pitch_offset = pitch;
	*mean_roll_offset = roll;
	*mean_g = imu_data.accel[2];

	for(int mean_samples = 1; mean_samples < 100; mean_samples++) {
		sem_wait(IMU_sem);
		get_latest_imu(&imu_data);
		pitch = imu_data.euler[0] * RAD_TO_DEG;
		roll = imu_data.euler[1] * RAD_TO_DEG;

		*mean_pitch_offset = (double)(mean_samples - 1) / ((double)mean_samples) * *mean_pitch_offset + pitch / ((double)mean_samples);
		*mean_roll_offset = (double)(mean_samples - 1) / ((double)mean_samples) * *mean_roll_offset + roll / ((double)mean_samples);
		*mean_g = (double)(mean_samples - 1) / ((double)mean_samples) * *mean_g + imu_data.accel[2]/ ((double)mean_samples);
	}

	printio("Pitch offset:\t\t\t%.4f", *mean_pitch_offset);
	printio("Roll offset:\t\t\t%.4f", *mean_roll_offset);
	printio("Gravitational constant:\t%.4f", *mean_g);

	FILE* calibration_file = fopen("pitch_roll_offset.cal", "w");

	if (calibration_file == NULL){
		fprintf(stderr, "Could not open a calibration file!\n");
	}

	fprintf(calibration_file, "%lf %lf %lf", *mean_pitch_offset, *mean_roll_offset, *mean_g);
	
	fclose(calibration_file);
}

void load_offset(double* mean_pitch_offset, double* mean_roll_offset, double* mean_g) {
	FILE* calibration_file = fopen("pitch_roll_offset.cal", "r");

	if (calibration_file == NULL){
		fprintf(stderr, "Could not open a calibration file! Use flag -c if you have not created one.\n");
	}

	fscanf(calibration_file, "%lf %lf %lf", mean_pitch_offset, mean_roll_offset, mean_g);
	
	fclose(calibration_file);

	printio("Pitch offset:\t\t%.4f", *mean_pitch_offset);
	printio("Roll offset:\t\t%.4f", *mean_roll_offset);
	printio("Gravitational constant:\t%.4f", *mean_g);
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
			printio("New K_p is set %lf", K_p);
			printio("New K_r is set %lf", K_r);
			printio("New T_d_p is set %lf", T_d_p);
			printio("New T_d_r is set %lf", T_d_r);
			printio("New K_y is set %lf", K_y);
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

void rate_PID(esc_input_t *esc_input, double thr, controller_data_t* controller_data){
	p_rate_error = pitch_rate_ref - p_rate;
	r_rate_error = roll_rate_ref - r_rate;
	y_rate_error = yaw_rate_ref - y_rate;


	controller_data->rate_errors[0] = r_rate_error;
	controller_data->rate_errors[1] = p_rate_error;
	controller_data->rate_errors[2]= y_rate_error;


	v_p = K_pr_p * p_rate_error;
	v_r = K_pr_r * r_rate_error;
	v_y = K_pr_y * y_rate_error;

	controller_data->rate_pid[0] = v_r;
	controller_data->rate_pid[1] = v_p;
	controller_data->rate_pid[2] = v_y;
	
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
}


void angle_PID(double* pitch_ref, double* roll_ref, double* yaw_ref, controller_data_t* controller_data){
	double p_angle_error = clip(*pitch_ref - pitch, -70.0, 70.0);
	double r_angle_error = clip(*roll_ref - roll, -70.0, 70.0);
	I_a_p = I_a_p + K_ia_p * TS * p_angle_error;
	I_a_r = I_a_r + K_ia_r * TS * r_angle_error;
	//I_a_y = I_a_y + K_ia_y * TS * y_angle_error;
	
	I_a_p = (abs_fnc(p_angle_error) < MIN_ERROR) ? 0.0 : I_a_p;
	I_a_r = (abs_fnc(r_angle_error) < MIN_ERROR) ? 0.0 : I_a_r;
	//I_a_y = (abs_fnc(y_angle_error) < MIN_ERROR) ? 0.0 : I_a_y;

	I_a_p = clip(I_a_p, -MAX_I, MAX_I);
	I_a_r = clip(I_a_r, -MAX_I, MAX_I);
	//I_a_y = clip(I_a_y, -MAX_I, MAX_I);
	P_a_p = K_pa_p * p_angle_error;
	P_a_r = K_pa_r * r_angle_error;

	pitch_rate_ref =  P_a_p + I_a_p;
 	roll_rate_ref =  P_a_r + I_a_r;
	yaw_rate_ref = *yaw_ref;

	// Update data in controller struct for plotting
	controller_data->angle_errors[0] = r_angle_error;
	controller_data->angle_errors[1] = p_angle_error;
	controller_data->k_angle_pid[1] = K_pa_r*r_angle_error;
	controller_data->k_angle_pid[1] = K_pa_p*p_angle_error;
	controller_data -> integral_pid[0] = I_a_r;
	controller_data -> integral_pid[1] = I_a_p;
	controller_data -> rate_refs[0] = roll_rate_ref;
	controller_data -> rate_refs[1] = pitch_rate_ref;
	controller_data -> rate_refs[2] = yaw_rate_ref;

}

// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
int flight_main(sem_t *IMU_sem, controller_data_t * controller_data){
	flight_mode_t flight_mode = FLIGHT;
	// int samples = 0;
	// double mean_z_speed = 0;
	// double mean_z_acc = 0;

	double mean_roll_offset = 0;
	double mean_pitch_offset = 0;
	double mean_g = 0;
	
	double pitch_ref = 0;
	double roll_ref = 0;
	double yaw_ref = 0;
	// double z_speed = 0;
	double thr = 0; // normalized throttle

	int sent_arming_error = 0;
	int sent_dsm_prevent_descent_waring = 0;
	int sent_imu_wake_warning = 0;

	//Calibrate
	u_int64_t time_since_wake = 0;

	if(calibrate) calibrate_IMU(IMU_sem, &mean_pitch_offset, &mean_roll_offset, &mean_g);
	else  {
		load_offset(&mean_pitch_offset, &mean_roll_offset, &mean_g);
		set_warning("IMU waking up. It is not recommended to fly within 30 seconds of startup.");
		time_since_wake = rc_nanos_since_epoch();
		sent_imu_wake_warning = 1;
	}

	sleep(1);

	if(rc_dsm_ch_normalized(5) > 0.7) {
		set_error("Drone must be disarmed at startup.");

		while(rc_get_state() != EXITING && rc_dsm_ch_normalized(5) > 0.7) rc_usleep(1000000);
		
		resolve_error();
	} else {
		printio("Disarmed");
	}

	while (rc_get_state() != EXITING){
		if(sent_imu_wake_warning && time_since_wake + 30000000000 < rc_nanos_since_epoch()) {
			resolve_warning();
			sent_imu_wake_warning = 0;
		}
		
		if(!armed && rc_dsm_ch_normalized(1) > 0.0000000001 && rc_dsm_ch_normalized(5) > 0.7) {
			if(!sent_arming_error) {
				set_error("Could not arm, throttle was %.2lf", rc_dsm_ch_normalized(1));
				sent_arming_error = 1;
			}

			armed = 0;
		} else if (rc_dsm_ch_normalized(5) > 0.7){
			if(sent_arming_error) {
				resolve_error();
				sent_arming_error = 0;
			}

			armed = 1;
			set_armed(1);//IO_THEAD
			rc_led_set(RC_LED_RED, 1);
		} else {
			if(sent_arming_error) {
				resolve_error();
				sent_arming_error = 0;
			}
			
			armed = 0;
			set_armed(0);//IO_THREAD
			rc_led_set(RC_LED_RED, 0);
		}

		// Height regulator. UNUSED
		if (rc_dsm_ch_normalized(7) > 0.7){ 
			// if (const_alt_active == 0)
			// {
			// 	get_latest_baro(&baro_data);
			// 	alt_ref = baro_data.bmp_data.alt_m;
			// 	printio("Baro ref alt: %8.2f ", baro_data.bmp_data.alt_m);
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
		
		// Asymmetrical synchronization with the DSM data.
		sem_wait(IMU_sem);
		get_latest_imu(&imu_data);

		pitch = imu_data.euler[0] * RAD_TO_DEG - mean_pitch_offset;
		roll = imu_data.euler[1] * RAD_TO_DEG - mean_roll_offset;
		yaw = imu_data.gyro[2];

		p_rate = imu_data.gyro[0];
		r_rate = imu_data.gyro[1];
		y_rate = imu_data.gyro[2];

		controller_data->angles[0] = roll;
		controller_data->angles[1] = pitch;
		//controller_data->angles[2] = yaw;

		controller_data->rates[0] = p_rate;
		controller_data->rates[1] = r_rate;
		controller_data->rates[2] = y_rate;

		//z_speed += TS*(imu_data.accel[2]-G);
		//printf("\nz_speed: %lf\n", z_speed);
		
		// // Tried to estimate velocity but it performed lowsy. 
		// LP_filter(imu_data.accel[2], &mean_z_acc, 0.1);
		// z_speed += TS*(dead_zone(mean_z_acc-mean_g, 0.1));
		// update_value(z_speed);

		switch (flight_mode) {
		case DESCEND:
			if(!lost_dsm_connection() && rc_dsm_ch_normalized(6) <= 0.7){
				flight_mode = FLIGHT;
				printio("Enter flight mode");
			} else {
				if (rc_dsm_nanos_since_last_packet() > 20000000000) {
					flight_mode = LANDED;
					printio("Enter landed mode");
					armed = 0;
					thr = 0;
				}
				// if  (samples == 0){
				// 	mean_z_speed = z_speed;
				// 	samples++;
				// } else if (samples == 30){
				// 	if (abs_fnc(mean_z_speed) < 0.4) {
				// 		flight_mode = LANDED;
				// 		printio("Enter landed mode");
				// 		armed = 0;
				// 		thr = 0;
				// 	}
				// 	printio("mean_z_speed: %lf",mean_z_speed);
				// 	samples=0;
				// } else {
				// 	mean_z_speed = (double)(samples - 1) / ((double)samples)
				// 	 	* mean_z_speed + z_speed / ((double)samples);
				// 	samples++; 
				// }
			}
			break;

		case LANDED:
			armed = 0;
			thr = 0;

			if(!lost_dsm_connection() && rc_dsm_ch_normalized(6) <= 0.7){
				flight_mode = FLIGHT;
				printio("Enter flight mode");
			} 
			break;

		case FLIGHT:
			if(lost_dsm_connection() || rc_dsm_ch_normalized(6) > 0.7) {
				if(thr <= 0.0000000001) {
					if(!sent_dsm_prevent_descent_waring) {
						set_warning("Won't enable DEDCEND-mode beacuse thottle was 0.0");
						sent_dsm_prevent_descent_waring = 1;
					}
				} else {
					flight_mode = DESCEND;
					printio("Enter descend mode");
					thr = DESCEND_THR;
					pitch_ref = 0;
					roll_ref = 0;
					yaw_ref = 0;
				}
			} else {
				if(sent_dsm_prevent_descent_waring) {
					resolve_warning();
					sent_dsm_prevent_descent_waring = 0;
				}

				thr = rc_dsm_ch_normalized(1);
				update_value(thr);
				// bound the signal to the escs
				if (thr < -0.1) thr = -0.1;
				if (thr > 1.0) thr = 1.0;
			}
			break;
		}

		if (manual_mode){
			manual_output(&esc_input, thr);
		} else {		
			// update_PID_param();
			// PID_controller(TS, &esc_input, thr);
			angle_PID(&pitch_ref, &roll_ref, &yaw_ref, controller_data);
			rate_PID(&esc_input,thr, controller_data);
		}

		if(armed) {
			rc_servo_send_esc_pulse_normalized(1, esc_input.u_1);
			rc_servo_send_esc_pulse_normalized(3, esc_input.u_2);
			rc_servo_send_esc_pulse_normalized(5, esc_input.u_3);
			rc_servo_send_esc_pulse_normalized(7, esc_input.u_4);
		} else {
			rc_servo_send_esc_pulse_normalized(0, 0);
		}

		// sleep roughly enough to maintain FREQUENCY
		rc_usleep(1000000 / FREQUENCY);
	}

	return 0;
}

void *flight_thread_func(sem_t *IMU_sem){
	flight_thread_ret_val = flight_main(IMU_sem, &controller_data);
	if (flight_thread_ret_val == -1){
		rc_set_state(EXITING);
	}
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 0);
	return (void *)&flight_thread_ret_val;
}