#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mp_controller.h"
#include "mp_driver.h"
#include "time.h"
#include "log.h"
#include "utils.h"

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

// Radio panel
enum {
    MP_ALT_UP_CMD_MSG,
    MP_ALT_DN_CMD_MSG = MP_ALT_UP_CMD_MSG + 1,
    MP_VS_UP_CMD_MSG,
    MP_VS_DN_CMD_MSG = MP_VS_UP_CMD_MSG + 1,
    MP_IAS_UP_CMD_MSG,
    MP_IAS_DN_CMD_MSG = MP_IAS_UP_CMD_MSG + 1,
    MP_HDG_UP_CMD_MSG,
    MP_HDG_DN_CMD_MSG = MP_HDG_UP_CMD_MSG + 1,
    MP_CRS_UP_CMD_MSG,
    MP_CRS_DN_CMD_MSG = MP_CRS_UP_CMD_MSG + 1
};

enum {
	MP_KNOB_ALT,
	MP_KNOB_VS,
	MP_KNOB_IAS,
	MP_KNOB_HDG,
	MP_KNOB_CRS
};

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
static struct thread_data *gPtrThreadData;
static unsigned char buf[MP_IN_BUF_SIZE];
static unsigned char writeBuf[MP_OUT_BUF_SIZE];
static char tmp[100];

/* MULTI PANEL Command Refs */
XPLMCommandRef gMpAltDnCmdRef = NULL;
XPLMCommandRef gMpAltUpCmdRef = NULL;
XPLMCommandRef gMpVrtclSpdDnCmdRef = NULL;
XPLMCommandRef gMpVrtclSpdUpCmdRef = NULL;
XPLMCommandRef gMpAsDnCmdRef = NULL;
XPLMCommandRef gMpAsUpCmdRef = NULL;
XPLMCommandRef gMpHdgDnCmdRef = NULL;
XPLMCommandRef gMpHdgUpCmdRef = NULL;
XPLMCommandRef gMpObsHsiDnCmdRef = NULL;
XPLMCommandRef gMpObsHsiUpCmdRef = NULL;

/* MULTI PANEL Data Refs */
XPLMDataRef gMpAltHoldFtDataRef = NULL;
XPLMDataRef gMpVrtVelDataRef = NULL;
XPLMDataRef gMpArspdDataRef = NULL;
XPLMDataRef gMpHdgMagDataRef = NULL;
XPLMDataRef gMpHsiObsDegMagPltDataRef = NULL;
XPLMDataRef gMpHsiSrcSelPltDataRef = NULL;
XPLMDataRef gMpNav1CrsDefMagPltDataRef = NULL;
XPLMDataRef gMpNav2CrsDefMagPltDataRef = NULL;
XPLMDataRef gMpGpsCourseDataRef = NULL;

static uint32_t gMpAlt = 0;
static int32_t gMpVS = 0;
static uint32_t gMpIAS = 0;
static uint32_t gMpHDG = 0;
static uint32_t gMpCRS = 0;
static int gIndicatorKnob = MP_KNOB_ALT;


int MultiPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void *            inRefcon) {
//	XPLMDebugString("-> CP: MultiPanelCommandHandler: start.\n");
//	char Buffer[256];
//	sprintf(Buffer,"Cmdh handler: 0x%08x, %d, 0x%08x\n", inCommand, inPhase, inRefcon);
//	XPLMDebugString(Buffer);
	int status = CMD_PASS_EVENT;

 switch ((int)(inRefcon)) {
		case MP_ALT_UP_CMD_MSG:
		case MP_ALT_DN_CMD_MSG:
		    gMpAlt = round(XPLMGetDataf(gMpAltHoldFtDataRef));
			break;
		case MP_VS_UP_CMD_MSG:
		case MP_VS_DN_CMD_MSG:
			gMpVS = round(XPLMGetDataf(gMpVrtVelDataRef));
			break;
		case MP_IAS_UP_CMD_MSG:
		case MP_IAS_DN_CMD_MSG:
			gMpIAS = round(XPLMGetDataf(gMpArspdDataRef));
			break;
		case MP_HDG_UP_CMD_MSG:
		case MP_HDG_DN_CMD_MSG:
			gMpHDG = round(XPLMGetDataf(gMpHdgMagDataRef));
			break;
		case MP_CRS_UP_CMD_MSG:
		case MP_CRS_DN_CMD_MSG:
			gMpCRS = round(XPLMGetDataf(gMpHsiSrcSelPltDataRef));
			break;
		default:
			break;
 }

 return status;
}


inline void mp_led_update(uint32_t x, uint32_t y, uint32_t s, uint8_t m[]) {
    m[0] = 0x00;
    m[1] = ((x >> 16) & 0xFF);
    m[2] = ((x >> 12) & 0xFF);
    m[3] = ((x >>  8) & 0xFF);
    m[4] = ((x >>  4) & 0xFF);
    m[5] = ((x >>  0) & 0xFF);
    m[6] = s;
    m[7] = ((y >> 12) & 0xFF);
    m[8] = ((y >>  8) & 0xFF);
    m[9] = ((y >>  4) & 0xFF);
    m[10] = ((y >>  0) & 0xFF);
}

int mp_process(uint32_t msg) {
//    sprintf(tmp, "-> CP: mp_controller.mp_process: msg: %d\n", msg);
//	XPLMDebugString(tmp);
	int res = 0;
    uint32_t readKnob = msg & MP_READ_KNOB_MODE_MASK;
    uint32_t readTuning = msg & MP_READ_TUNING_MASK;

    if (readKnob == MP_READ_KNOB_ALT) {
    	gIndicatorKnob = MP_KNOB_ALT;
    } else if (readKnob == MP_READ_KNOB_VS) {
    	gIndicatorKnob = MP_KNOB_VS;
    } else if (readKnob == MP_READ_KNOB_IAS) {
    	gIndicatorKnob = MP_KNOB_IAS;
    } else if (readKnob == MP_READ_KNOB_HDG) {
    	gIndicatorKnob = MP_KNOB_HDG;
    } else if (readKnob == MP_READ_KNOB_CRS) {
    	gIndicatorKnob = MP_KNOB_CRS;
    }

    if (readTuning) {
		if (readTuning == MP_READ_TUNING_RIGHT) {
			if (readKnob == MP_READ_KNOB_ALT) {
				XPLMCommandOnce(gMpAltUpCmdRef);
			} else if (readKnob == MP_READ_KNOB_VS) {
				XPLMCommandOnce(gMpVrtclSpdUpCmdRef);
			} else if (readKnob == MP_READ_KNOB_IAS) {
				XPLMCommandOnce(gMpAsUpCmdRef);
			} else if (readKnob == MP_READ_KNOB_HDG) {
				XPLMCommandOnce(gMpHdgUpCmdRef);
			} else if (readKnob == MP_READ_KNOB_CRS) {
				XPLMCommandOnce(gMpObsHsiUpCmdRef);
			}
		} else if (readTuning == MP_READ_TUNING_LEFT) {
			if (readKnob == MP_READ_KNOB_ALT) {
				XPLMCommandOnce(gMpAltDnCmdRef);
			} else if (readKnob == MP_READ_KNOB_VS) {
				XPLMCommandOnce(gMpVrtclSpdDnCmdRef);
			} else if (readKnob == MP_READ_KNOB_IAS) {
				XPLMCommandOnce(gMpAsDnCmdRef);
			} else if (readKnob == MP_READ_KNOB_HDG) {
				XPLMCommandOnce(gMpHdgDnCmdRef);
			} else if (readKnob == MP_READ_KNOB_CRS) {
				XPLMCommandOnce(gMpObsHsiDnCmdRef);
			}
		}
    }
	return res;
}

void mp_update_datarefs() {
    gMpAlt = round(XPLMGetDataf(gMpAltHoldFtDataRef));
    gMpVS = round(XPLMGetDataf(gMpVrtVelDataRef));
    gMpIAS = round(XPLMGetDataf(gMpArspdDataRef));
    gMpHDG = round(XPLMGetDataf(gMpHdgMagDataRef));
    gMpCRS = round(XPLMGetDataf(gMpHsiObsDegMagPltDataRef));
}

void mp_init() {
	XPLMDebugString("-> CP: mp_controller.mp_init.\n");

    gMpAltDnCmdRef          = XPLMFindCommand(sMP_ALTITUDE_DOWN_CR);
    gMpAltUpCmdRef          = XPLMFindCommand(sMP_ALTITUDE_UP_CR);
    gMpVrtclSpdDnCmdRef     = XPLMFindCommand(sMP_VERTICAL_SPEED_DOWN_CR);
    gMpVrtclSpdUpCmdRef     = XPLMFindCommand(sMP_VERTICAL_SPEED_UP_CR);
    gMpAsDnCmdRef           = XPLMFindCommand(sMP_AIRSPEED_DOWN_CR);
    gMpAsUpCmdRef           = XPLMFindCommand(sMP_AIRSPEED_UP_CR);
    gMpHdgDnCmdRef          = XPLMFindCommand(sMP_HEADING_DOWN_CR);
    gMpHdgUpCmdRef          = XPLMFindCommand(sMP_HEADING_UP_CR);
    gMpObsHsiDnCmdRef       = XPLMFindCommand(sMP_OBS_HSI_DOWN_CR);
    gMpObsHsiUpCmdRef       = XPLMFindCommand(sMP_OBS_HSI_UP_CR);


    XPLMRegisterCommandHandler(gMpAltDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_ALT_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMpAltUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_ALT_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpVrtclSpdDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_VS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMpVrtclSpdUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_VS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpAsDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_IAS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMpAsUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_IAS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpHdgDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_HDG_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMpHdgUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_HDG_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpObsHsiDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_CRS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMpObsHsiUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_CRS_DN_CMD_MSG);

    gMpAltHoldFtDataRef         = XPLMFindDataRef(sMP_ALTITUDE_DIAL_FT_DR);
    gMpVrtVelDataRef            = XPLMFindDataRef(sMP_VVI_DIAL_FPM_DR);
    gMpArspdDataRef             = XPLMFindDataRef(sMP_AIRSPEED_DR);
    gMpHdgMagDataRef            = XPLMFindDataRef(sMP_HEADING_DIAL_DEG_MAG_PILOT_DR);
    gMpHsiObsDegMagPltDataRef   = XPLMFindDataRef(sMP_HSI_OBS_DEG_MAG_PILOT_DR);
    gMpHsiSrcSelPltDataRef      = XPLMFindDataRef(sMP_HSI_SOURCE_SELECT_PILOT_DR);
    gMpNav1CrsDefMagPltDataRef  = XPLMFindDataRef(sMP_NAV1_COURSE_DEG_MAG_PILOT_DR);
    gMpNav2CrsDefMagPltDataRef  = XPLMFindDataRef(sMP_NAV2_COURSE_DEG_MAG_PILOT_DR);
    gMpGpsCourseDataRef         = XPLMFindDataRef(sMP_GPS_COURSE_DR);

    gMpAlt = round(XPLMGetDataf(gMpAltHoldFtDataRef));
    gMpVS = round(XPLMGetDataf(gMpVrtVelDataRef));
    gMpIAS = round(XPLMGetDataf(gMpArspdDataRef));
    gMpHDG = round(XPLMGetDataf(gMpHdgMagDataRef));
    gMpCRS = round(XPLMGetDataf(gMpHsiObsDegMagPltDataRef));
}

void *mpRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	int inReportBytesCount = 0;

	mp_init();

	gPtrThreadData = (struct thread_data *) ptr_thread_data;

	mp_panel_open();
	XPLMDebugString("-> CP: mp_controller.mp_run: panel opened.\n");

	last_mainloop_idle = sys_time_clock_get_time_usec();
	// while stop == 0 calculate position.
	while (gPtrThreadData->stop == 0) {
		long loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// Read panel for new messages. CRITICAL FAST 100 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(10000, COUNTER5, loop_start_time)) {
			// read/write board
			counter++;
			inReportBytesCount = mp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
//			    sprintf(tmp, "-> CP: mp_controller.run: msg %d: %#0x,%#0x,%#0x\n", counter, buf[2], buf[1], buf[0]);
//				XPLMDebugString(tmp);
				counter2++;
				uint32_t msg = 0;
				msg += buf[2] << 16;
				msg += buf[1] << 8;
				msg += buf[0];
				mp_process(msg);
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// Update Panel. NON-CRITICAL 20 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER6, loop_start_time)) {
			// Update local DataRefs.
			mp_update_datarefs();
			// update Panel.
			inReportBytesCount = mp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
//			    sprintf(tmp, "-> CP: mp_controller.run: msg %d: %#0x,%#0x,%#0x\n", counter, buf[2], buf[1], buf[0]);
//				XPLMDebugString(tmp);
				uint32_t msg = 0;
				msg += buf[2] << 16;
				msg += buf[1] << 8;
				msg += buf[0];
				mp_process(msg);
			}
			uint32_t pos_negativ = MP_LED_PLUS_SIGN;
			switch (gIndicatorKnob) {
			case MP_KNOB_ALT:
			case MP_KNOB_VS:
				tmp1 = dec2bcd(gMpAlt, 5);
				tmp2 = dec2bcd(abs(gMpVS), 4);
				if (gMpVS >= 0) {
					pos_negativ = MP_LED_PLUS_SIGN;
				} else {
					pos_negativ = MP_LED_MINUS_SIGN;
				}
				break;
			case MP_KNOB_IAS:
				tmp1 = dec2bcd(gMpIAS, 3);
				tmp2 = 0;
				break;
			case MP_KNOB_HDG:
				tmp1 = dec2bcd(gMpHDG, 3);
				tmp2 = 0;
				break;
			case MP_KNOB_CRS:
				tmp1 = dec2bcd(gMpCRS, 3);
				tmp2 = 0;
				break;
			default:
				break;
			}
			mp_led_update(tmp1, tmp2, pos_negativ, writeBuf);
			mp_panel_write(writeBuf);
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= 100000) {
			XPLMDebugString("-> CP: mp_controller.run: CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
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
	mp_panel_close();
	pthread_exit(NULL);
	return 0;
}

