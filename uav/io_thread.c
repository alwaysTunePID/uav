#include "io_thread.h"
#include "imu.h"
#include "battery_thread.h"
#include <stdio.h>
#include <robotcontrol.h>
#include <unistd.h>

#define GREEN "\033[1;32m"
#define YELLOW "\033[01;33m"
#define RED "\033[0;31m"
#define RESET_COLOR "\033[0m"

#define SPACE_BUFFER "                                                  "

static int io_thread_ret_val;
static uint64_t dsm_nanos = 0;

static int warnings = 0;
static int errors = 0;

int io_main(void) {
	sleep(1);

    while (rc_get_state() != EXITING) {
        char color[8];

        if(battery_data.voltage > 11.5) color = GREEN;
        else if (battery_data.voltage > 11) color = YELLOW;
        else color = RED;

        if(dsm_nanos == 0) printf("\r  Battery voltage: %s%lf%s V%s", color, battery_data.voltage, RESET_COLOR, SPACE_BUFFER);
        else printf("\r  Battery voltage: %s%lf%s V%s Seconds since last DSM packet: %.2f", color, battery_data.voltage, RESET_COLOR, SPACE_BUFFER, dsm_nanos/1000000000.0);

		fflush(stdout);
		
        rc_usleep(500000);
    }

    return 0;
}

void calibration_sleep() {
    sleep(1);
    char loading[30] = "#.............................";
    for(int i = 1; i < 30 && rc_get_state() != EXITING; i++) {
        loading[i] = '#';
        printf("\r  IMU waking up [%s] %d s / 30 s", loading, (i+1));
        fflush(stdout);
        sleep(1);
    }

    printf("\r  IMU waking up [%s] Done!       \n", loading);
    fflush(stdout);
}

void dsm_signal_loss_warning(uint64_t time_ns) {
    dsm_nanos = time_ns;
}

void dsm_signal_restored() {
    dsm_nanos = 0;
}

void *io_thread_func(void) {
    io_thread_ret_val = io_main();
    if (io_thread_ret_val ==-1) {
        rc_set_state(EXITING);
    }
	
    return (void*)&io_thread_ret_val;
}
