#ifndef IMU_H
#define IMU_H

extern "C"
{ 
#include <robotcontrol.h>
}
#include <semaphore.h>
//int get_oldest_mpu_data(rc_mpu_data_t*);

// a wraper struct of  rc_mpu_data_t with the extra time field
typedef struct imu_entry_t {
  	uint64_t time_ns;
	double accel[3];
	double gyro[3];
	double mag[3];
	double temp;
	double euler[3];
} imu_entry_t;


int initialize_imu();
int finalize_imu();
int log_imu();
int get_latest_imu(imu_entry_t *imu);
void sample_imu(sem_t*);

#endif //IMU_H
