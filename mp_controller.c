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

// Multi panel
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
    MP_CRS_DN_CMD_MSG = MP_CRS_UP_CMD_MSG + 1,
    MP_AP_FD_UP_CMD_MSG,
    MP_AP_FD_DN_CMD_MSG = MP_AP_FD_UP_CMD_MSG + 1,
    MP_AP_FD_OFF_CMD_MSG,
    MP_AP_HEADING_CMD_MSG,
    MP_AP_NAV_CMD_MSG,
    MP_AP_IAS_CMD_MSG,
    MP_AP_ALT_CMD_MSG,
    MP_AP_VS_CMD_MSG,
    MP_AP_APR_CMD_MSG,
    MP_AP_REV_CMD_MSG,
    MP_PITCH_TRIM_DN_CMD_MSG,
    MP_PITCH_TRIM_UP_CMD_MSG = MP_PITCH_TRIM_DN_CMD_MSG + 1,
    MP_PITCH_TRIM_TAKEOFF_CMD_MSG,
    MP_FLAPS_DN_CMD_MSG,
    MP_FLAPS_UP_CMD_MSG
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
static struct mp_thread_data *gPtrThreadData;
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

XPLMCommandRef gMpApFlightDirectorOnCmdRef = NULL;
XPLMCommandRef gMpApFlightDirectorOffCmdRef = NULL;
XPLMCommandRef gMpApHeadingCmdRef = NULL;
XPLMCommandRef gMpApNAVCmdRef = NULL;
XPLMCommandRef gMpApIASCmdRef = NULL;
XPLMCommandRef gMpApAltCmdRef = NULL;
XPLMCommandRef gMpApVsCmdRef = NULL;
XPLMCommandRef gMpApAprCmdRef = NULL;
XPLMCommandRef gMpApRevBackCourseCmdRef = NULL;

XPLMCommandRef gMpPitchTrimDnCmdRef = NULL;
XPLMCommandRef gMpPitchTrimUpCmdRef = NULL;
XPLMCommandRef gMpPitchTrimTakeoffCmdRef = NULL;
XPLMCommandRef gMpFlapsDnCmdRef = NULL;
XPLMCommandRef gMpFlapsUpCmdRef = NULL;


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

XPLMDataRef gMpAltDataRef = NULL;
XPLMDataRef gMpVSDataRef = NULL;
XPLMDataRef gMpIASDataRef = NULL;
XPLMDataRef gMpHDGDataRef = NULL;
XPLMDataRef gMpCRSDataRef = NULL;

static uint32_t gMpAlt = 0;
static int32_t gMpVS = 0;
static uint32_t gMpIAS = 0;
static uint32_t gMpHDG = 0;
static uint32_t gMpCRS = 0;

XPLMDataRef gMpApMasterDataRef = NULL;
XPLMDataRef gMpApHdgDataRef = NULL;
XPLMDataRef gMpApNavDataRef = NULL;
XPLMDataRef gMpApIASDataRef = NULL;
XPLMDataRef gMpApAltDataRef = NULL;
XPLMDataRef gMpApVsDataRef = NULL;
XPLMDataRef gMpApAprDataRef = NULL;
XPLMDataRef gMpApRevDataRef = NULL;
XPLMDataRef gMpApAutopilotStateDataRef = NULL;
XPLMDataRef gMpApAutopilotModeDataRef = NULL;

static uint32_t gMpApButton = 0;
static uint32_t gMpHdgButton = 0;
static uint32_t gMpNavButton = 0;
static uint32_t gMpIasButton = 0;
static uint32_t gMpAltButton = 0;
static uint32_t gMpVsButton = 0;
static uint32_t gMpAprButton = 0;
static uint32_t gMpRevButton = 0;
static uint32_t gMpATButtonState = 0;
static uint32_t gMpAutopilotState = 0;
static uint32_t gMpAutopilotMode = 0;

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
		case MP_AP_FD_UP_CMD_MSG:
		case MP_AP_FD_DN_CMD_MSG:
		case MP_AP_FD_OFF_CMD_MSG:
			gMpApButton = XPLMGetDatai(gMpApMasterDataRef);
			break;
		case MP_AP_HEADING_CMD_MSG:
			gMpHdgButton = XPLMGetDatai(gMpApHdgDataRef);
			break;
		case MP_AP_NAV_CMD_MSG:
			gMpNavButton = XPLMGetDatai(gMpApNavDataRef);
			break;
		case MP_AP_IAS_CMD_MSG:
			gMpIasButton = XPLMGetDatai(gMpApIASDataRef);
			break;
		case MP_AP_ALT_CMD_MSG:
			gMpAltButton = XPLMGetDatai(gMpApAltDataRef);
			break;
		case MP_AP_VS_CMD_MSG:
			gMpVsButton = XPLMGetDatai(gMpApVsDataRef);
			break;
		case MP_AP_APR_CMD_MSG:
			gMpAprButton = XPLMGetDatai(gMpApAprDataRef);
			break;
		case MP_AP_REV_CMD_MSG:
			gMpRevButton = XPLMGetDatai(gMpApRevDataRef);
			break;
		case MP_PITCH_TRIM_DN_CMD_MSG:
		case MP_PITCH_TRIM_UP_CMD_MSG:
		case MP_PITCH_TRIM_TAKEOFF_CMD_MSG:
			break;
		case MP_FLAPS_DN_CMD_MSG:
		case MP_FLAPS_UP_CMD_MSG:
			break;
		default:
			break;
 }
 return status;
}


inline void mp_led_update(uint32_t x, uint32_t y, uint32_t s, uint32_t buttons, uint8_t m[]) {
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
    m[11] = ((buttons >>  0) & 0xFF);
}

int mp_process(uint32_t msg) {
//    sprintf(tmp, "-> CP: mp_controller.mp_process: msg: %d\n", msg);
//	XPLMDebugString(tmp);
	int res = 0;
    uint32_t readKnob = msg & MP_READ_KNOB_MODE_MASK;
    uint32_t readTuning = msg & MP_READ_TUNING_MASK;
    uint32_t readButtons = msg & MP_READ_BTNS_MASK;
    uint32_t readFlaps = msg & MP_READ_FLAPS_MASK;
    uint32_t readTrim = msg & MP_READ_TRIM_MASK;
    uint32_t readAutoThrottle = msg & MP_READ_THROTTLE_MASK;

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

    if (readButtons) {
    	if (readButtons == MP_READ_AP_BTN) {
    		if (gMpAutopilotMode == 0) {
        		XPLMCommandOnce(gMpApFlightDirectorOnCmdRef);
        		// Check Autothrottle
        		if (readAutoThrottle == MP_READ_THROTTLE_ON) {
        			if (!((gMpAutopilotState) & 0x0001)) {
        				// 0x0001 Autothrottle Arm
        				XPLMCommandKeyStroke(xplm_key_otto_atr);
        			}
        		}
    		} else {
        		XPLMCommandOnce(gMpApFlightDirectorOffCmdRef);
    		}
    	} else if (readButtons == MP_READ_HDG_BTN) {
    		XPLMCommandOnce(gMpApHeadingCmdRef);
    	} else if (readButtons == MP_READ_NAV_BTN) {
    		XPLMCommandOnce(gMpApNAVCmdRef);
    	} else if (readButtons == MP_READ_IAS_BTN) {
    		XPLMCommandOnce(gMpApIASCmdRef);
    	} else if (readButtons == MP_READ_ALT_BTN) {
    		XPLMCommandOnce(gMpApAltCmdRef);
    	} else if (readButtons == MP_READ_VS_BTN) {
    		XPLMCommandOnce(gMpApVsCmdRef);
    	} else if (readButtons == MP_READ_APR_BTN) {
    		XPLMCommandOnce(gMpApAprCmdRef);
    	} else if (readButtons == MP_READ_REV_BTN) {
    		XPLMCommandOnce(gMpApRevBackCourseCmdRef);
    	}
    }

    if (readFlaps) {
    	if (readFlaps == MP_READ_FLAPS_DN) {
    		XPLMCommandOnce(gMpFlapsDnCmdRef);
    	} else if (readFlaps == MP_READ_FLAPS_UP) {
    		XPLMCommandOnce(gMpFlapsUpCmdRef);
    	}
    }

    if (readTrim) {
    	if (readTrim == MP_READ_TRIM_DOWN) {
    		XPLMCommandOnce(gMpPitchTrimDnCmdRef);
    	} else if (readTrim == MP_READ_TRIM_UP) {
    		XPLMCommandOnce(gMpPitchTrimUpCmdRef);
    	}
    }

    if (gMpATButtonState != readAutoThrottle) {
    	gMpATButtonState = readAutoThrottle;
		// Autothrottle Changed
		if (readAutoThrottle == MP_READ_THROTTLE_ON) {
			//XPLMCommandOnce(gMcpApAutoThrottleOnCmdRef);
			if (!((gMpAutopilotState) & 0x0001)) {
				// 0x0001 Autothrottle Arm
				XPLMCommandKeyStroke(xplm_key_otto_atr);
			}
		} else {
			//XPLMCommandOnce(gMcpApAutoThrottleOffCmdRef);
			if ((gMpAutopilotState) & 0x0001) {
				// 0x0001 Autothrottle Disarm
				XPLMCommandKeyStroke(xplm_key_otto_atr);
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

    gMpApButton = XPLMGetDatai(gMpApMasterDataRef);
    gMpHdgButton = XPLMGetDatai(gMpApHdgDataRef);
    gMpNavButton = XPLMGetDatai(gMpApNavDataRef);
    gMpIasButton = XPLMGetDatai(gMpApIASDataRef);
    gMpAltButton = XPLMGetDatai(gMpApAltDataRef);
    gMpVsButton = XPLMGetDatai(gMpApVsDataRef);
    gMpAprButton = XPLMGetDatai(gMpApAprDataRef);
    gMpRevButton = XPLMGetDatai(gMpApRevDataRef);
    gMpAutopilotState = XPLMGetDatai(gMpApAutopilotStateDataRef);
	gMpAutopilotMode = XPLMGetDatai(gMpApAutopilotModeDataRef);
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

    gMpApFlightDirectorOnCmdRef  = XPLMFindCommand(sMP_SERVOS_AND_FLIGHT_DIR_ON_CR);
    gMpApFlightDirectorOffCmdRef = XPLMFindCommand(sMP_SERVOS_AND_FLIGHT_DIR_OFF_CR);
    gMpApHeadingCmdRef           = XPLMFindCommand(sMP_AP_HEADING_CR);
    gMpApNAVCmdRef               = XPLMFindCommand(sMP_AP_NAV_CR);
    gMpApIASCmdRef               = XPLMFindCommand(sMP_AP_LEVEL_CHANGE_CR);
    gMpApAltCmdRef               = XPLMFindCommand(sMP_AP_ALTITUDE_HOLD_CR);
    gMpApVsCmdRef                = XPLMFindCommand(sMP_AP_VERTICAL_SPEED_CR);
    gMpApAprCmdRef               = XPLMFindCommand(sMP_AP_APPROACH_CR);
    gMpApRevBackCourseCmdRef     = XPLMFindCommand(sMP_AP_BACK_COURSE_CR);

    XPLMRegisterCommandHandler(gMpApFlightDirectorOnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_FD_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApFlightDirectorOffCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_FD_OFF_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApHeadingCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_HEADING_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApNAVCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_NAV_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApIASCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_IAS_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApAltCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_ALT_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApVsCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_VS_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApAprCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_APR_CMD_MSG);
    XPLMRegisterCommandHandler(gMpApRevBackCourseCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_AP_REV_CMD_MSG);

    gMpPitchTrimDnCmdRef         = XPLMFindCommand(sMP_PITCH_TRIM_DOWN_CR);
    gMpPitchTrimUpCmdRef         = XPLMFindCommand(sMP_PITCH_TRIM_UP_CR);
    gMpPitchTrimTakeoffCmdRef    = XPLMFindCommand(sMP_PITCH_TRIM_TAKEOFF_CR);
    gMpFlapsDnCmdRef             = XPLMFindCommand(sMP_FLAPS_DOWN_CR);
    gMpFlapsUpCmdRef             = XPLMFindCommand(sMP_FLAPS_UP_CR);

    XPLMRegisterCommandHandler(gMpPitchTrimDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_PITCH_TRIM_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpPitchTrimUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_PITCH_TRIM_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMpPitchTrimTakeoffCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_PITCH_TRIM_TAKEOFF_CMD_MSG);
    XPLMRegisterCommandHandler(gMpFlapsDnCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_FLAPS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMpFlapsUpCmdRef, MultiPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MP_FLAPS_UP_CMD_MSG);

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

    gMpApMasterDataRef = XPLMFindDataRef(sMP_AP_FLIGHT_DIR_MODE_DR);
    gMpApHdgDataRef = XPLMFindDataRef(sMP_AP_HEADING_STATUS_DR);
    gMpApNavDataRef = XPLMFindDataRef(sMP_AP_SPEED_STATUS_DR);
    gMpApIASDataRef = XPLMFindDataRef(sMP_AP_HEADING_STATUS_DR);
    gMpApAltDataRef = XPLMFindDataRef(sMP_AP_ALTITUDE_HOLD_STATUS_DR);
    gMpApVsDataRef = XPLMFindDataRef(sMP_AP_VVI_STATUS_DR);
    gMpApAprDataRef = XPLMFindDataRef(sMP_AP_APPROACH_STATUS_DR);
    gMpApRevDataRef = XPLMFindDataRef(sMP_AP_BACKCOURSE_STATUS_DR);
    gMpApAutopilotStateDataRef = XPLMFindDataRef(sMP_AUTOPILOT_STATE_DR);
    gMpApAutopilotModeDataRef = XPLMFindDataRef(sMP_AUTOPILOT_MODE_DR);

    gMpApButton = XPLMGetDatai(gMpApMasterDataRef);
    gMpHdgButton = XPLMGetDatai(gMpApHdgDataRef);
    gMpNavButton = XPLMGetDatai(gMpApNavDataRef);
    gMpIasButton = XPLMGetDatai(gMpApIASDataRef);
    gMpAltButton = XPLMGetDatai(gMpApAltDataRef);
    gMpVsButton = XPLMGetDatai(gMpApVsDataRef);
    gMpAprButton = XPLMGetDatai(gMpApAprDataRef);
    gMpRevButton = XPLMGetDatai(gMpApRevDataRef);
    gMpAutopilotState = XPLMGetDatai(gMpApAutopilotStateDataRef);
    gMpAutopilotMode = XPLMGetDatai(gMpApAutopilotModeDataRef);
}

void *mpRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	uint32_t buttons = 0;
	int inReportBytesCount = 0;

#if IBM
		Sleep(SLEEP_TIME);
#else
		usleep(SLEEP_TIME);
#endif

	mp_init();

	gPtrThreadData = (struct mp_thread_data *) ptr_thread_data;

	int result = mp_panel_open();
	if (result < 0) {
		XPLMDebugString("-> CP: mp_controller.mpRun: shutdown thread.\n");
		pthread_exit(NULL);
		return 0;
	}
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
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: mp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				}
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
		/// Update Panel. CRITICAL 20 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(50000, COUNTER6, loop_start_time)) {
			// Update local DataRefs.
			mp_update_datarefs();
			// update Panel.
			inReportBytesCount = mp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: mp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				}
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
			buttons = 0x00;
			if (gMpApButton) {
				buttons |= 0x01;
			}
			if (gMpHdgButton) {
				buttons |= 0x02;
			}
			if (gMpNavButton) {
				buttons |= 0x04;
			}
			if (gMpIasButton) {
				buttons |= 0x08;
			}
			if (gMpAltButton) {
				buttons |= 0x10;
			}
			if (gMpVsButton) {
				buttons |= 0x20;
			}
			if (gMpAprButton) {
				buttons |= 0x40;
			}
			if (gMpRevButton) {
				buttons |= 0x80;
			}
			mp_led_update(tmp1, tmp2, pos_negativ, buttons, writeBuf);
			mp_panel_write(writeBuf);
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= MAX_DELAY_TIME) {
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

