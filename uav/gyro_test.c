/**
 * @file rc_altitude.c
 * @example    rc_altitude
 *
 * Borrowed from:
 * @author     James Strawson
 * @date       3/14/2018
 */

#include <stdio.h>
#include <signal.h>
#include <math.h> // for M_PI
#include <rc/math/kalman.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <rc/bmp.h>
#include <rc/mpu.h>


#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE     200     // hz
#define DT              (1.0/SAMPLE_RATE)
#define GYRO_LP_TC     20*DT   // fast LP filter for gyro
#define PRINT_HZ        10
#define BMP_RATE_DIV    10      // optionally sample bmp less frequently than mpu

static int running = 0;
static rc_mpu_data_t mpu_data;
static int step = 0;
static rc_vector_t u = RC_VECTOR_INITIALIZER;
static rc_vector_t y = RC_VECTOR_INITIALIZER;
static rc_filter_t gyr_lpx = RC_FILTER_INITIALIZER;
static rc_filter_t gyr_lpy = RC_FILTER_INITIALIZER;
static rc_filter_t gyr_lpz = RC_FILTER_INITIALIZER;


// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
	running = 0;
	return;
}


static void __dmp_handler(void)
{
	int i;
	double gyro_vec[3];
	static int bmp_sample_counter = 0;

	// make copy of gyroeration reading before rotating
	for (i = 0; i < 3; i++) gyro_vec[i] = mpu_data.dmp_TaitBryan[i];
	// rotate gyro vector
	//rc_quaternion_rotate_vector_array(gyro_vec, mpu_data.dmp_quat);

	
	if (step == 0) {
		rc_filter_prefill_inputs(&gyr_lpx, gyro_vec[0]);
		rc_filter_prefill_outputs(&gyr_lpx, gyro_vec[0]);
		rc_filter_prefill_inputs(&gyr_lpy, gyro_vec[1]);
		rc_filter_prefill_outputs(&gyr_lpy, gyro_vec[1]);
		rc_filter_prefill_inputs(&gyr_lpz, gyro_vec[2]);
		rc_filter_prefill_outputs(&gyr_lpz, gyro_vec[2]);

		step = 1;
	}
	

	// calculate gyro... and smooth it just a tad
	rc_filter_march(&gyr_lpx, gyro_vec[0]);
	rc_filter_march(&gyr_lpy, gyro_vec[1]);
	rc_filter_march(&gyr_lpz, gyro_vec[2]);



	return;
}



int main(void)
{
	rc_mpu_config_t mpu_conf;

	// initialize the little LP filter to take out gyr noise
	if (rc_filter_first_order_lowpass(&gyr_lpx, DT, GYRO_LP_TC)) return -1;
	if (rc_filter_first_order_lowpass(&gyr_lpy, DT, GYRO_LP_TC)) return -1;
	if (rc_filter_first_order_lowpass(&gyr_lpz, DT, GYRO_LP_TC)) return -1;

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;


        // if gyro isn't calibrated, run the calibration routine
        if(!rc_mpu_is_gyro_calibrated()){
                printf("Gyro not calibrated, automatically starting calibration routine\n");
                printf("Let your MiP sit still on a firm surface\n");
                rc_mpu_calibrate_gyro_routine(mpu_conf);
        }

	// init DMP
	printf("initializing DMP\n");
	mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
	mpu_conf.dmp_fetch_accel_gyro = 1;
//	mpu_conf.orient = ORIENTATION_X_FORWARD | ORIENTATION_Z_DOWN | ORIENTATION_Y_UP;
//	mpu_conf.orient = ORIENTATION_X_UP;
//	mpu_conf.orient = ORIENTATION_Y_UP;
//	mpu_conf.orient = ORIENTATION_Z_DOWN;
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;

	// wait for dmp to settle then start filter callback
	printf("waiting for sensors to settle");
	fflush(stdout);
	rc_usleep(3000000);
	rc_mpu_set_dmp_callback(__dmp_handler);

	// print a header
	printf("\r\n");
	printf(" Pitch |");
	printf(" Roll |");
	printf(" Yaw |");
	printf("\n");

	//now just wait, print_data will run
	while (running) {
		rc_usleep(1000000 / PRINT_HZ);

		printf("\r");
		printf("%7.2f|", RAD_TO_DEG * gyr_lpy.newest_output);
		printf("%7.2f|", RAD_TO_DEG * gyr_lpx.newest_output);
		printf("%7.2f|", RAD_TO_DEG * gyr_lpz.newest_output);

		fflush(stdout);
	}
	printf("\n");

	rc_mpu_power_off();
	rc_bmp_power_off();
	return 0;
}

