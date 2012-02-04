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
#include "defs.h"
#include "time.h"
#include "log.h"
#include "utils.h"

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

// Radio panel
enum RP_COMMANDS_MAP {
	RP_CMD_STDBY_COM1_FINE_DOWN = 123,
	RP_CMD_STDBY_COM1_FINE_UP = 124,
	RP_CMD_STDBY_COM1_COARSE_DOWN = 125,
	RP_CMD_STDBY_COM1_COARSE_UP = 126,
	RP_CMD_ACTV_COM1_FINE_DOWN,
	RP_CMD_ACTV_COM1_FINE_UP,
	RP_CMD_ACTV_COM1_COARSE_DOWN,
	RP_CMD_ACTV_COM1_COARSE_UP,
	RP_CMD_COM1_STANDBY_FLIP,
	RP_CMD_COM2_STANDBY_FLIP
};

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
struct rp_thread_data *gPtrThreadData;
static unsigned char buf[RP_IN_BUF_SIZE];
static unsigned char writeBuf[RP_OUT_BUF_SIZE];
static char tmp[100];


/* RADIO PANEL Command Refs */
XPLMCommandRef gRpStdbyCOM1FineDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM1FineUpCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM1CoarseDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM1CoarseUpCmdRef = NULL;
XPLMCommandRef gRpActvCOM1FineDownCmdRef = NULL;
XPLMCommandRef gRpActvCOM1FineUpCmdRef = NULL;
XPLMCommandRef gRpActvCOM1CoarseDownCmdRef = NULL;
XPLMCommandRef gRpActvCOM1CoarseUpCmdRef = NULL;
XPLMCommandRef gRpCOM1StbyFlipCmdRef = NULL;
XPLMCommandRef gRpCOM2StbyFlipCmdRef = NULL;

/* RADIO PANEL Data Refs */
XPLMDataRef gRpCOM1FreqHzDataRef = NULL;
XPLMDataRef gRpCOM1StdbyFreqHzDataRef = NULL;

uint32_t gRpCOM1StbyFreq = 0;


int RadioPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void *            inRefcon) {
//	XPLMDebugString("-> CP: RadioPanelCommandHandler: start.\n");
//	char Buffer[256];
//	sprintf(Buffer,"Cmdh handler: 0x%08x, %d, 0x%08x\n", inCommand, inPhase, inRefcon);
//	XPLMDebugString(Buffer);
	int status = CMD_PASS_EVENT;

 switch ((int)(inRefcon)) {
		case RP_CMD_STDBY_COM1_FINE_DOWN:
		case RP_CMD_STDBY_COM1_FINE_UP:
		case RP_CMD_STDBY_COM1_COARSE_DOWN:
		case RP_CMD_STDBY_COM1_COARSE_UP:
			XPLMDebugString("-> CP: RadioPanelCommandHandler: COM1 changed.\n");
			gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
			break;
		case RP_CMD_ACTV_COM1_FINE_DOWN:
		case RP_CMD_ACTV_COM1_FINE_UP:
		case RP_CMD_ACTV_COM1_COARSE_DOWN:
		case RP_CMD_ACTV_COM1_COARSE_UP:
			XPLMDebugString("-> CP: RadioPanelCommandHandler: COM1 changed.\n");
			gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
			break;
		case RP_CMD_COM1_STANDBY_FLIP:
			XPLMDebugString("-> CP: RadioPanelCommandHandler: COM1 fliped.\n");
			gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
			break;
		default:
			break;
 }

 return status;
}


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

void rp_process_coarse_right(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gRpStdbyCOM1CoarseUpCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		break;
	case 0x000004:
	case 0x000200:
		break;
	case 0x000008:
	case 0x000400:
		break;
	case 0x000010:
	case 0x000800:
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		break;
	default:
		break;
	}
}

void rp_process_coarse_left(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gRpStdbyCOM1CoarseDownCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		break;
	case 0x000004:
	case 0x000200:
		break;
	case 0x000008:
	case 0x000400:
		break;
	case 0x000010:
	case 0x000800:
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		break;
	default:
		break;
	}
}

void rp_process_fine_right(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gRpStdbyCOM1FineUpCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		break;
	case 0x000004:
	case 0x000200:
		break;
	case 0x000008:
	case 0x000400:
		break;
	case 0x000010:
	case 0x000800:
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		break;
	default:
		break;
	}
}

void rp_process_fine_left(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gRpStdbyCOM1FineDownCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		break;
	case 0x000004:
	case 0x000200:
		break;
	case 0x000008:
	case 0x000400:
		break;
	case 0x000010:
	case 0x000800:
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		break;
	default:
		break;
	}
}

void rp_process_switch(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gRpCOM1StbyFlipCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gRpCOM2StbyFlipCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		break;
	case 0x000008:
	case 0x000400:
		break;
	case 0x000010:
	case 0x000800:
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		break;
	default:
		break;
	}
}

int rp_process(uint32_t msg) {
    sprintf(tmp, "-> CP: rp_controller.rp_process: msg: %d\n", msg);
	XPLMDebugString(tmp);
	int res = 0;
    uint32_t upperKnob = msg & RP_READ_UPPER_KNOB_MODE_MASK;
    uint32_t lowerKnob = msg & RP_READ_LOWER_KNOB_MODE_MASK;
    uint32_t upperFineTuning = msg & RP_READ_UPPER_FINE_TUNING_MASK;
    uint32_t upperCoarseTuning = msg & RP_READ_UPPER_COARSE_TUNING_MASK;
    uint32_t lowerFineTuning = msg & RP_READ_LOWER_FINE_TUNING_MASK;
    uint32_t lowerCoarseTuning = msg & RP_READ_LOWER_COARSE_TUNING_MASK;
    uint32_t upperStby = msg & RP_READ_UPPER_ACT_STBY;
    uint32_t lowerStby = msg & RP_READ_LOWER_ACT_STBY;

    if (upperCoarseTuning || upperFineTuning) {
		if (upperCoarseTuning == RP_READ_UPPER_COARSE_RIGHT) {
			XPLMDebugString("-> CP: rp_controller.rp_process RP_READ_UPPER_COARSE_RIGHT.\n");
			rp_process_coarse_right(upperKnob);
		} else if (upperCoarseTuning == RP_READ_UPPER_COARSE_LEFT) {
			XPLMDebugString("-> CP: rp_controller.rp_process RP_READ_UPPER_COARSE_LEFT.\n");
			rp_process_coarse_left(upperKnob);
		} else if (upperFineTuning == RP_READ_UPPER_FINE_RIGHT) {
			XPLMDebugString("-> CP: rp_controller.rp_process RP_READ_UPPER_FINE_RIGHT.\n");
			rp_process_fine_right(upperKnob);
		} else if (upperFineTuning == RP_READ_UPPER_FINE_LEFT) {
			XPLMDebugString("-> CP: rp_controller.rp_process RP_READ_UPPER_FINE_LEFT.\n");
			rp_process_fine_left(upperKnob);
		}
    }
    if (upperStby) {
    	rp_process_switch(upperKnob);
    }
    if (lowerCoarseTuning || lowerFineTuning) {
		if (lowerCoarseTuning == RP_READ_LOWER_COARSE_RIGHT) {
			rp_process_coarse_right(lowerKnob);
		} else if (lowerCoarseTuning == RP_READ_LOWER_COARSE_LEFT) {
			rp_process_coarse_left(lowerKnob);
		} else if (lowerFineTuning == RP_READ_LOWER_FINE_RIGHT) {
			rp_process_fine_right(lowerKnob);
		} else if (lowerFineTuning == RP_READ_LOWER_FINE_LEFT) {
			rp_process_fine_left(lowerKnob);
		}
    }
    if (lowerStby) {
    	rp_process_switch(lowerKnob);
    }
    return res;
}

void rp_update_datarefs() {
    gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
}

void rp_init() {
	XPLMDebugString("-> CP: rp_controller.rp_init.\n");
	gRpStdbyCOM1FineDownCmdRef         = XPLMFindCommand(sRP_STDBY_COM1_FINE_DOWN_CR);
	gRpStdbyCOM1FineUpCmdRef           = XPLMFindCommand(sRP_STDBY_COM1_FINE_UP_CR);
	gRpStdbyCOM1CoarseDownCmdRef       = XPLMFindCommand(sRP_STDBY_COM1_COARSE_DOWN_CR);
	gRpStdbyCOM1CoarseUpCmdRef         = XPLMFindCommand(sRP_STDBY_COM1_COARSE_UP_CR);
	gRpActvCOM1FineDownCmdRef         = XPLMFindCommand(sRP_ACTV_COM1_FINE_DOWN_CR);
	gRpActvCOM1FineUpCmdRef           = XPLMFindCommand(sRP_ACTV_COM1_FINE_UP_CR);
	gRpActvCOM1CoarseDownCmdRef       = XPLMFindCommand(sRP_ACTV_COM1_COARSE_DOWN_CR);
	gRpActvCOM1CoarseUpCmdRef         = XPLMFindCommand(sRP_ACTV_COM1_COARSE_UP_CR);
	gRpCOM1StbyFlipCmdRef              = XPLMFindCommand(sRP_COM1_STBY_FLIP_CR);
	gRpCOM2StbyFlipCmdRef              = XPLMFindCommand(sRP_COM2_STBY_FLIP_CR);

    XPLMRegisterCommandHandler(gRpStdbyCOM1FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM1_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyCOM1FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM1_FINE_UP);
    XPLMRegisterCommandHandler(gRpStdbyCOM1CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyCOM1CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM1_COARSE_UP);
    XPLMRegisterCommandHandler(gRpActvCOM1FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM1_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpActvCOM1FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM1_FINE_UP);
    XPLMRegisterCommandHandler(gRpActvCOM1CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpActvCOM1CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM1_COARSE_UP);
    XPLMRegisterCommandHandler(gRpCOM1StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_COM1_STANDBY_FLIP);
    XPLMRegisterCommandHandler(gRpCOM2StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_COM2_STANDBY_FLIP);

	gRpCOM1FreqHzDataRef        = XPLMFindDataRef(sRP_COM1_FREQ_HZ_DR);
    gRpCOM1StdbyFreqHzDataRef   = XPLMFindDataRef(sRP_COM1_STDBY_FREQ_HZ_DR);

    gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
}

void *rpRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	uint32_t tmp3 = 0;
	uint32_t tmp4 = 0;
	int inReportBytesCount = 0;

	rp_init();

	gPtrThreadData = (struct rp_thread_data *) ptr_thread_data;

	panel_open();
	XPLMDebugString("-> CP: rp_controller.run: panel opened.\n");

	last_mainloop_idle = sys_time_clock_get_time_usec();
	// while stop == 0 calculate position.
	while (gPtrThreadData->stop == 0) {
		long loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// Read panel for new messages. CRITICAL FAST 100 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(10000, COUNTER1, loop_start_time)) {
			// read/write board
			counter++;
			tmp1 = dec2bcd(counter % 100000, 5);
			//tmp2 = dec2bcd(counter % 1000000, 5);
			tmp2 = dec2bcd(gRpCOM1StbyFreq, 5);
			tmp3 = dec2bcd(counter2 % 1000000, 5);
			tmp4 = dec2bcd(counter2 % 1000000, 5);
			rp_upper_led_update(tmp1, tmp2, tmp3, tmp4, writeBuf);
			inReportBytesCount = panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: pp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				}
				counter2++;
				uint32_t msg = 0;
				msg += buf[2] << 16;
				msg += buf[1] << 8;
				msg += buf[0];
				rp_process(msg);
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// Update Panel. NON-CRITICAL 20 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER2, loop_start_time)) {
			// Update local DataRefs.
			rp_update_datarefs();
			// update Panel.
			inReportBytesCount = panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: pp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				}
				uint32_t msg = 0;
				msg += buf[2] << 16;
				msg += buf[1] << 8;
				msg += buf[0];
				rp_process(msg);
			}
			panel_write(writeBuf);
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= 100000) {
			XPLMDebugString("-> CP: rp_controller.run: CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		} else {
			//writeConsole(0, 0, "CPU LOAD OK.");
		}

		// wait 1 milliseconds
#if IBM
		Sleep(SLEEP_TIME);
#else
		usleep(SLEEP_TIME);
#endif
	}
	panel_close();
	pthread_exit(NULL);
	return 0;
}

