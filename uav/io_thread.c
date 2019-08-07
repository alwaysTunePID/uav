#include "io_thread.h"
#include "imu.h"
#include "battery_thread.h"
#include "queue.h"
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

static int pval = 0.0;

int block_main = 0;

int io_main(void) {
	sleep(1);

    while (rc_get_state() != EXITING) {
        rc_usleep(500000);

        if(block_main) continue;

        char color[8];
        char status[23];

        //Set battery color
        if(battery_data.voltage > 11.5 || battery_data.voltage < 0.1) strcpy(color, GREEN);//Will show 0.0 V when powered by USB.
        else if (battery_data.voltage > 11) strcpy(color, YELLOW);
        else strcpy(color, RED);
        
        //Check and set current status
        if(errors > 0 || strcmp(color, RED) == 0) sprintf(status, "%s[ERROR]%s", RED, RESET_COLOR);
        else if (warnings > 0 ||strcmp(color, YELLOW) == 0) sprintf(status, "%s[WARN]%s", YELLOW, RESET_COLOR);
        else sprintf(status, "%s[OK]%s", GREEN, RESET_COLOR);

        //Print
        if(dsm_nanos == 0) printf("\r  %s Battery voltage: %s%.2lf%s V%s Z-speed: %.3lf", status, color, battery_data.voltage, RESET_COLOR, SPACE_BUFFER, pval);
        else if (dsm_nanos >= 18446744073) printf("\r  %s Battery voltage: %s%.2lf%s V DSM has not been connected. Z-speed: %.3lf", status, color, battery_data.voltage, RESET_COLOR, pval);
        else printf("\r  %s Battery voltage: %s%.2lf%s V Seconds since last DSM packet: %.2f Z-speed: %.3lf", status, color, battery_data.voltage, RESET_COLOR, dsm_nanos/1000000000.0, pval);

		fflush(stdout);
    }

    return 0;
}

void calibration_sleep() {
    block_main++;
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
    block_main--;
}

void dsm_signal_loss_warning(uint64_t time_ns) {
    if(dsm_nanos == 0) errors++;
    dsm_nanos = time_ns;
}

void dsm_signal_restored() {
    if(dsm_nanos != 0) errors--;
    dsm_nanos = 0;
}

void update_value(double value) {
    pval = value;
}

void *io_thread_func(void) {
    io_thread_ret_val = io_main();
    if (io_thread_ret_val ==-1) {
        rc_set_state(EXITING);
    }
	
    return (void*)&io_thread_ret_val;
}
