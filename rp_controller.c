#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "rp_controller.h"
#include "rp_driver.h"
#include "time.h"
#include "log.h"
#include "utils.h"
#include "XPLMUtilities.h"

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
struct thread_data *gPtrThreadData;
static unsigned char buf[RP_IN_BUF_SIZE];
static unsigned char writeBuf[RP_OUT_BUF_SIZE];


inline void rp_upper_led_update(uint32_t x, uint32_t y, uint32_t z, uint32_t r, uint8_t m[]) {
    m[0] = 0x00;
    m[1] = ((x >> 16) & 0xFF);
    m[2] = ((x >> 12) & 0xFF);
    m[3] = ((x >>  8) & 0xFF);
    m[4] = ((x >>  4) & 0xFF);
    m[5] = ((x >>  0) & 0xFF);
    m[6] = ((y >> 16) & 0xFF);
    m[7] = ((y >> 12) & 0xFF);
    m[8] = ((y >>  8) & 0xFF);
    m[9] = ((y >>  4) & 0xFF);
    m[10] = ((y >>  0) & 0xFF);
    m[11] = ((z >> 16) & 0xFF);
    m[12] = ((z >> 12) & 0xFF);
    m[13] = ((z >>  8) & 0xFF);
    m[14] = ((z >>  4) & 0xFF);
    m[15] = ((z >>  0) & 0xFF);
    m[16] = ((r >> 16) & 0xFF);
    m[17] = ((r >> 12) & 0xFF);
    m[18] = ((r >>  8) & 0xFF);
    m[19] = ((r >>  4) & 0xFF);
    m[20] = ((r >>  0) & 0xFF);
    m[21] = 0x00;
    m[22] = 0x00;
}

void *run(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	uint32_t tmp3 = 0;
	uint32_t tmp4 = 0;
	int inReportBytesCount = 0;

	gPtrThreadData = (struct thread_data *) ptr_thread_data;

	panel_open();
	XPLMDebugString("-> CP: rp_controller.run: panel opened.\n");

	last_mainloop_idle = sys_time_clock_get_time_usec();
	// while stop == 0 calculate position.
	while (gPtrThreadData->stop == 0) {
		long loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL FAST 200 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(5000, COUNTER3, loop_start_time)) {
			// read/write board
			counter++;
			tmp1 = dec2bcd(counter % 1000, 5);
			tmp2 = dec2bcd(counter % 1000000, 5);
			tmp3 = dec2bcd(counter2 % 1000000, 5);
			tmp4 = dec2bcd(counter2 % 1000000, 5);
			rp_upper_led_update(tmp1, tmp2, tmp3, tmp4, writeBuf);
			inReportBytesCount = panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				counter2++;
			}
			panel_write(writeBuf);
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 10 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(100000, COUNTER6, loop_start_time)) {
			//update_screen();
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= 100000) {
			writeLog("CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		} else {
			//writeConsole(0, 0, "CPU LOAD OK.");
		}

		// wait 1 milliseconds
#if IBM
		Sleep(1);
#else
		usleep(10);
#endif
	}
	panel_close();
	pthread_exit(NULL);
	return 0;
}

