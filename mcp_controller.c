#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mcp_controller.h"
#include "mcp_driver.h"
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
    MCP_ALT_UP_CMD_MSG,
    MCP_ALT_DN_CMD_MSG = MCP_ALT_UP_CMD_MSG + 1,
    MCP_VS_UP_CMD_MSG,
    MCP_VS_DN_CMD_MSG = MCP_VS_UP_CMD_MSG + 1,
    MCP_IAS_UP_CMD_MSG,
    MCP_IAS_DN_CMD_MSG = MCP_IAS_UP_CMD_MSG + 1,
    MCP_HDG_UP_CMD_MSG,
    MCP_HDG_DN_CMD_MSG = MCP_HDG_UP_CMD_MSG + 1,
    MCP_CRS_UP_CMD_MSG,
    MCP_CRS_DN_CMD_MSG = MCP_CRS_UP_CMD_MSG + 1,
    MCP_AP_FD_UP_CMD_MSG,
    MCP_AP_FD_DN_CMD_MSG = MCP_AP_FD_UP_CMD_MSG + 1,
    MCP_AP_FD_OFF_CMD_MSG,
    MCP_AP_HEADING_CMD_MSG,
    MCP_AP_NAV_CMD_MSG,
    MCP_AP_IAS_CMD_MSG,
    MCP_AP_ALT_CMD_MSG,
    MCP_AP_VS_CMD_MSG,
    MCP_AP_APR_CMD_MSG,
    MCP_AP_REV_CMD_MSG,
    MCP_PITCH_TRIM_DN_CMD_MSG,
    MCP_PITCH_TRIM_UP_CMD_MSG = MCP_PITCH_TRIM_DN_CMD_MSG + 1,
    MCP_PITCH_TRIM_TAKEOFF_CMD_MSG,
    MCP_FLAPS_DN_CMD_MSG,
    MCP_FLAPS_UP_CMD_MSG
};

enum {
	MCP_KNOB_ALT,
	MCP_KNOB_VS,
	MCP_KNOB_IAS,
	MCP_KNOB_HDG,
	MCP_KNOB_CRS
};

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
static struct mcp_thread_data *gPtrThreadData;
static unsigned char buf[MCP_IN_BUF_SIZE];
static unsigned char writeBuf[MCP_OUT_BUF_SIZE];
static char tmp[100];

/* MULTI PANEL Command Refs */
XPLMCommandRef gMcpAltDnCmdRef = NULL;
XPLMCommandRef gMcpAltUpCmdRef = NULL;
XPLMCommandRef gMcpVrtclSpdDnCmdRef = NULL;
XPLMCommandRef gMcpVrtclSpdUpCmdRef = NULL;
XPLMCommandRef gMcpAsDnCmdRef = NULL;
XPLMCommandRef gMcpAsUpCmdRef = NULL;
XPLMCommandRef gMcpHdgDnCmdRef = NULL;
XPLMCommandRef gMcpHdgUpCmdRef = NULL;
XPLMCommandRef gMcpObsHsiDnCmdRef = NULL;
XPLMCommandRef gMcpObsHsiUpCmdRef = NULL;

XPLMCommandRef gMcpApFlightDirectorUpCmdRef = NULL;
XPLMCommandRef gMcpApFlightDirectorDnCmdRef = NULL;
XPLMCommandRef gMcpApFlightDirectorOffCmdRef = NULL;
XPLMCommandRef gMcpApHeadingCmdRef = NULL;
XPLMCommandRef gMcpApNAVCmdRef = NULL;
XPLMCommandRef gMcpApIASCmdRef = NULL;
XPLMCommandRef gMcpApAltCmdRef = NULL;
XPLMCommandRef gMcpApVsCmdRef = NULL;
XPLMCommandRef gMcpApAprCmdRef = NULL;
XPLMCommandRef gMcpApRevBackCourseCmdRef = NULL;

XPLMCommandRef gMcpPitchTrimDnCmdRef = NULL;
XPLMCommandRef gMcpPitchTrimUpCmdRef = NULL;
XPLMCommandRef gMcpPitchTrimTakeoffCmdRef = NULL;
XPLMCommandRef gMcpFlapsDnCmdRef = NULL;
XPLMCommandRef gMcpFlapsUpCmdRef = NULL;


/* MULTI PANEL Data Refs */
XPLMDataRef gMcpAltHoldFtDataRef = NULL;
XPLMDataRef gMcpVrtVelDataRef = NULL;
XPLMDataRef gMcpArspdDataRef = NULL;
XPLMDataRef gMcpHdgMagDataRef = NULL;
XPLMDataRef gMcpHsiObsDegMagPltDataRef = NULL;
XPLMDataRef gMcpHsiSrcSelPltDataRef = NULL;
XPLMDataRef gMcpNav1CrsDefMagPltDataRef = NULL;
XPLMDataRef gMcpNav2CrsDefMagPltDataRef = NULL;
XPLMDataRef gMcpGpsCourseDataRef = NULL;

XPLMDataRef gMcpAltDataRef = NULL;
XPLMDataRef gMcpVSDataRef = NULL;
XPLMDataRef gMcpIASDataRef = NULL;
XPLMDataRef gMcpHDGDataRef = NULL;
XPLMDataRef gMcpCRSDataRef = NULL;

static uint32_t gMcpAlt = 0;
static int32_t gMcpVS = 0;
static uint32_t gMcpIAS = 0;
static uint32_t gMcpHDG = 0;
static uint32_t gMcpCRS = 0;

XPLMDataRef gMcpApMasterDataRef = NULL;
XPLMDataRef gMcpApHdgDataRef = NULL;
XPLMDataRef gMcpApNavDataRef = NULL;
XPLMDataRef gMcpApIASDataRef = NULL;
XPLMDataRef gMcpApAltDataRef = NULL;
XPLMDataRef gMcpApVsDataRef = NULL;
XPLMDataRef gMcpApAprDataRef = NULL;
XPLMDataRef gMcpApRevDataRef = NULL;

static uint32_t gMcpApButton = 0;
static uint32_t gMcpHdgButton = 0;
static uint32_t gMcpNavButton = 0;
static uint32_t gMcpIasButton = 0;
static uint32_t gMcpAltButton = 0;
static uint32_t gMcpVsButton = 0;
static uint32_t gMcpAprButton = 0;
static uint32_t gMcpRevButton = 0;

static int gIndicatorKnob = MCP_KNOB_ALT;


int MainControlPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void *            inRefcon) {
//	XPLMDebugString("-> CP: MainControlPanelCommandHandler: start.\n");
//	char Buffer[256];
//	sprintf(Buffer,"Cmdh handler: 0x%08x, %d, 0x%08x\n", inCommand, inPhase, inRefcon);
//	XPLMDebugString(Buffer);
	int status = CMD_PASS_EVENT;

 switch ((int)(inRefcon)) {
		case MCP_ALT_UP_CMD_MSG:
		case MCP_ALT_DN_CMD_MSG:
		    gMcpAlt = round(XPLMGetDataf(gMcpAltHoldFtDataRef));
			break;
		case MCP_VS_UP_CMD_MSG:
		case MCP_VS_DN_CMD_MSG:
			gMcpVS = round(XPLMGetDataf(gMcpVrtVelDataRef));
			break;
		case MCP_IAS_UP_CMD_MSG:
		case MCP_IAS_DN_CMD_MSG:
			gMcpIAS = round(XPLMGetDataf(gMcpArspdDataRef));
			break;
		case MCP_HDG_UP_CMD_MSG:
		case MCP_HDG_DN_CMD_MSG:
			gMcpHDG = round(XPLMGetDataf(gMcpHdgMagDataRef));
			break;
		case MCP_CRS_UP_CMD_MSG:
		case MCP_CRS_DN_CMD_MSG:
			gMcpCRS = round(XPLMGetDataf(gMcpHsiSrcSelPltDataRef));
			break;
		case MCP_AP_FD_UP_CMD_MSG:
		case MCP_AP_FD_DN_CMD_MSG:
		case MCP_AP_FD_OFF_CMD_MSG:
			gMcpApButton = XPLMGetDatai(gMcpApMasterDataRef);
			break;
		case MCP_AP_HEADING_CMD_MSG:
			gMcpHdgButton = XPLMGetDatai(gMcpApHdgDataRef);
			break;
		case MCP_AP_NAV_CMD_MSG:
			gMcpNavButton = XPLMGetDatai(gMcpApNavDataRef);
			break;
		case MCP_AP_IAS_CMD_MSG:
			gMcpIasButton = XPLMGetDatai(gMcpApIASDataRef);
			break;
		case MCP_AP_ALT_CMD_MSG:
			gMcpAltButton = XPLMGetDatai(gMcpApAltDataRef);
			break;
		case MCP_AP_VS_CMD_MSG:
			gMcpVsButton = XPLMGetDatai(gMcpApVsDataRef);
			break;
		case MCP_AP_APR_CMD_MSG:
			gMcpAprButton = XPLMGetDatai(gMcpApAprDataRef);
			break;
		case MCP_AP_REV_CMD_MSG:
			gMcpRevButton = XPLMGetDatai(gMcpApRevDataRef);
			break;
		case MCP_PITCH_TRIM_DN_CMD_MSG:
		case MCP_PITCH_TRIM_UP_CMD_MSG:
		case MCP_PITCH_TRIM_TAKEOFF_CMD_MSG:
			break;
		case MCP_FLAPS_DN_CMD_MSG:
		case MCP_FLAPS_UP_CMD_MSG:
			break;
		default:
			break;
 }
 return status;
}


inline void mcp_led_update(uint32_t x, uint32_t y, uint32_t s, uint32_t buttons, uint8_t m[]) {
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

unsigned char mcp_get_digit(uint8_t digit) {
	switch (digit) {
	case 0:
		return 0x3F;
	case 1:
		return 0x06;
	case 2:
		return 0x5B;
	case 3:
		return 0x4F;
	case 4:
		return 0x66;
	case 5:
		return 0x6D;
	case 6:
		return 0x7D;
	case 7:
		return 0x07;
	case 8:
		return 0x7F;
	case 9:
		return 0x67;
	default:
		return 0x00;
	}
}

void mcp_update_display() {
	// course
	if (gMcpCRS != round(XPLMGetDataf(gMcpHsiObsDegMagPltDataRef))) {
		gMcpCRS = round(XPLMGetDataf(gMcpHsiObsDegMagPltDataRef));
		mcp_course_left[1] = mcp_get_digit(gMcpCRS / 100);
		mcp_course_left[2] = mcp_get_digit(gMcpCRS % 100 / 10);
		mcp_course_left[3] = mcp_get_digit(gMcpCRS % 10);
		mcp_panel_write(mcp_course_left);
	}

}

int mcp_process(uint32_t msg) {
//    sprintf(tmp, "-> CP: mcp_controller.mcp_process: msg: %d\n", msg);
//	XPLMDebugString(tmp);
	int res = 0;
    uint32_t readPushButtons = msg & MCP_READ_PUSH_BUTTONS_MASK;
    uint32_t readButtons = msg & MCP_READ_BUTTONS_MASK;

    if (readPushButtons) {
    	if (readPushButtons == MCP_READ_BTN_HEADING) {
    		XPLMCommandOnce(gMcpApHeadingCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_LNAV) {
    		XPLMCommandOnce(gMcpApNAVCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_IAS_MACH) {
    		XPLMCommandOnce(gMcpApIASCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_ALTITUDE) {
    		XPLMCommandOnce(gMcpApAltCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_V_S) {
    		XPLMCommandOnce(gMcpApVsCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_APP) {
    		XPLMCommandOnce(gMcpApAprCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_ALT_HLD) {
    		XPLMCommandOnce(gMcpApRevBackCourseCmdRef);
    	}
    }

    if (readButtons) {
    	if (readButtons == MCP_READ_BTN_F_D) {
    	    XPLMCommandOnce(gMcpApFlightDirectorDnCmdRef);
    	} else if (readButtons == MCP_READ_BTN_A_T_ARM) {
    		//XPLMCommandOnce(gMcpUpCmdRef);
    	}
    }

	return res;
}

int mcp_process_knob(uint32_t msg) {
//    sprintf(tmp, "-> CP: mcp_controller.mcp_process: msg: %d\n", msg);
//	XPLMDebugString(tmp);
	int res = 0;
    uint32_t readCourseLeft = msg & MCP_READ_KNOB_COURSE_L_MASK;
    uint32_t readISAMach = msg & MCP_READ_KNOB_HEADING_MASK;
    uint32_t readHeading = msg & MCP_READ_KNOB_IAS_MACH_MASK;
    uint32_t readAltitute = msg & MCP_READ_KNOB_VERTSPEED_MASK;
    uint32_t readVertSpeed = msg & MCP_READ_KNOB_ALTITUE_MASK;
    uint32_t readCourseRight = msg & MCP_READ_KNOB_COURSE_R_MASK;

    if (readCourseLeft) {
    	int c1 = (msg & 0x000000F0) >> 4;
    	int i;
//    	sprintf(tmp, "mcp_process_knob: c1: %d\n", c1);
//    	XPLMDebugString(tmp);
    	if (c1 < 8) {
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpObsHsiUpCmdRef);
    		}
    	} else {
    		for (i = 0; i < (16 - c1); i++) {
    			XPLMCommandOnce(gMcpObsHsiDnCmdRef);
    		}
    	}
    }

	return res;
}


void mcp_update_datarefs() {
    gMcpAlt = round(XPLMGetDataf(gMcpAltHoldFtDataRef));
    gMcpVS = round(XPLMGetDataf(gMcpVrtVelDataRef));
    gMcpIAS = round(XPLMGetDataf(gMcpArspdDataRef));
    gMcpHDG = round(XPLMGetDataf(gMcpHdgMagDataRef));
//    gMcpCRS = round(XPLMGetDataf(gMcpHsiObsDegMagPltDataRef));

    gMcpApButton = XPLMGetDatai(gMcpApMasterDataRef);
    gMcpHdgButton = XPLMGetDatai(gMcpApHdgDataRef);
    gMcpNavButton = XPLMGetDatai(gMcpApNavDataRef);
    gMcpIasButton = XPLMGetDatai(gMcpApIASDataRef);
    gMcpAltButton = XPLMGetDatai(gMcpApAltDataRef);
    gMcpVsButton = XPLMGetDatai(gMcpApVsDataRef);
    gMcpAprButton = XPLMGetDatai(gMcpApAprDataRef);
    gMcpRevButton = XPLMGetDatai(gMcpApRevDataRef);
}

void mcp_init() {
	XPLMDebugString("-> CP: mcp_controller.mcp_init.\n");

    gMcpAltDnCmdRef          = XPLMFindCommand(sMCP_ALTITUDE_DOWN_CR);
    gMcpAltUpCmdRef          = XPLMFindCommand(sMCP_ALTITUDE_UP_CR);
    gMcpVrtclSpdDnCmdRef     = XPLMFindCommand(sMCP_VERTICAL_SPEED_DOWN_CR);
    gMcpVrtclSpdUpCmdRef     = XPLMFindCommand(sMCP_VERTICAL_SPEED_UP_CR);
    gMcpAsDnCmdRef           = XPLMFindCommand(sMCP_AIRSPEED_DOWN_CR);
    gMcpAsUpCmdRef           = XPLMFindCommand(sMCP_AIRSPEED_UP_CR);
    gMcpHdgDnCmdRef          = XPLMFindCommand(sMCP_HEADING_DOWN_CR);
    gMcpHdgUpCmdRef          = XPLMFindCommand(sMCP_HEADING_UP_CR);
    gMcpObsHsiDnCmdRef       = XPLMFindCommand(sMCP_OBS_HSI_DOWN_CR);
    gMcpObsHsiUpCmdRef       = XPLMFindCommand(sMCP_OBS_HSI_UP_CR);

    XPLMRegisterCommandHandler(gMcpAltDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_ALT_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpAltUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_ALT_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpVrtclSpdDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_VS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpVrtclSpdUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_VS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpAsDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_IAS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpAsUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_IAS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpHdgDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_HDG_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpHdgUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_HDG_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpObsHsiDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_CRS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpObsHsiUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_CRS_DN_CMD_MSG);

    gMcpApFlightDirectorUpCmdRef  = XPLMFindCommand(sMCP_FDIR_SERVOS_UP_ONE_CR);
    gMcpApFlightDirectorDnCmdRef  = XPLMFindCommand(sMCP_FDIR_SERVOS_DOWN_ONE_CR);
    gMcpApFlightDirectorOffCmdRef = XPLMFindCommand(sMCP_SERVOS_AND_FLIGHT_DIR_OFF_CR);
    gMcpApHeadingCmdRef           = XPLMFindCommand(sMCP_AP_HEADING_CR);
    gMcpApNAVCmdRef               = XPLMFindCommand(sMCP_AP_NAV_CR);
    gMcpApIASCmdRef               = XPLMFindCommand(sMCP_AP_LEVEL_CHANGE_CR);
    gMcpApAltCmdRef               = XPLMFindCommand(sMCP_AP_ALTITUDE_HOLD_CR);
    gMcpApVsCmdRef                = XPLMFindCommand(sMCP_AP_VERTICAL_SPEED_CR);
    gMcpApAprCmdRef               = XPLMFindCommand(sMCP_AP_APPROACH_CR);
    gMcpApRevBackCourseCmdRef     = XPLMFindCommand(sMCP_AP_BACK_COURSE_CR);

    XPLMRegisterCommandHandler(gMcpApFlightDirectorUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApFlightDirectorDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApFlightDirectorOffCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_OFF_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApHeadingCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_HEADING_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApNAVCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_NAV_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApIASCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_IAS_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAltCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_ALT_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApVsCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_VS_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAprCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_APR_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApRevBackCourseCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_REV_CMD_MSG);

    gMcpPitchTrimDnCmdRef         = XPLMFindCommand(sMCP_PITCH_TRIM_DOWN_CR);
    gMcpPitchTrimUpCmdRef         = XPLMFindCommand(sMCP_PITCH_TRIM_UP_CR);
    gMcpPitchTrimTakeoffCmdRef    = XPLMFindCommand(sMCP_PITCH_TRIM_TAKEOFF_CR);
    gMcpFlapsDnCmdRef             = XPLMFindCommand(sMCP_FLAPS_DOWN_CR);
    gMcpFlapsUpCmdRef             = XPLMFindCommand(sMCP_FLAPS_UP_CR);

    XPLMRegisterCommandHandler(gMcpPitchTrimDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_PITCH_TRIM_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpPitchTrimUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_PITCH_TRIM_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpPitchTrimTakeoffCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_PITCH_TRIM_TAKEOFF_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpFlapsDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_FLAPS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpFlapsUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_FLAPS_UP_CMD_MSG);

    gMcpAltHoldFtDataRef         = XPLMFindDataRef(sMCP_ALTITUDE_DIAL_FT_DR);
    gMcpVrtVelDataRef            = XPLMFindDataRef(sMCP_VVI_DIAL_FPM_DR);
    gMcpArspdDataRef             = XPLMFindDataRef(sMCP_AIRSPEED_DR);
    gMcpHdgMagDataRef            = XPLMFindDataRef(sMCP_HEADING_DIAL_DEG_MAG_PILOT_DR);
    gMcpHsiObsDegMagPltDataRef   = XPLMFindDataRef(sMCP_HSI_OBS_DEG_MAG_PILOT_DR);
    gMcpHsiSrcSelPltDataRef      = XPLMFindDataRef(sMCP_HSI_SOURCE_SELECT_PILOT_DR);
    gMcpNav1CrsDefMagPltDataRef  = XPLMFindDataRef(sMCP_NAV1_COURSE_DEG_MAG_PILOT_DR);
    gMcpNav2CrsDefMagPltDataRef  = XPLMFindDataRef(sMCP_NAV2_COURSE_DEG_MAG_PILOT_DR);
    gMcpGpsCourseDataRef         = XPLMFindDataRef(sMCP_GPS_COURSE_DR);

    gMcpAlt = round(XPLMGetDataf(gMcpAltHoldFtDataRef));
    gMcpVS = round(XPLMGetDataf(gMcpVrtVelDataRef));
    gMcpIAS = round(XPLMGetDataf(gMcpArspdDataRef));
    gMcpHDG = round(XPLMGetDataf(gMcpHdgMagDataRef));
//    gMcpCRS = round(XPLMGetDataf(gMcpHsiObsDegMagPltDataRef));

    gMcpApMasterDataRef = XPLMFindDataRef(sMCP_AP_FLIGHT_DIR_MODE_DR);
    gMcpApHdgDataRef = XPLMFindDataRef(sMCP_AP_HEADING_STATUS_DR);
    gMcpApNavDataRef = XPLMFindDataRef(sMCP_AP_SPEED_STATUS_DR);
    gMcpApIASDataRef = XPLMFindDataRef(sMCP_AP_HEADING_STATUS_DR);
    gMcpApAltDataRef = XPLMFindDataRef(sMCP_AP_ALTITUDE_HOLD_STATUS_DR);
    gMcpApVsDataRef = XPLMFindDataRef(sMCP_AP_VVI_STATUS_DR);
    gMcpApAprDataRef = XPLMFindDataRef(sMCP_AP_APPROACH_STATUS_DR);
    gMcpApRevDataRef = XPLMFindDataRef(sMCP_AP_BACKCOURSE_STATUS_DR);

    gMcpApButton = XPLMGetDatai(gMcpApMasterDataRef);
    gMcpHdgButton = XPLMGetDatai(gMcpApHdgDataRef);
    gMcpNavButton = XPLMGetDatai(gMcpApNavDataRef);
    gMcpIasButton = XPLMGetDatai(gMcpApIASDataRef);
    gMcpAltButton = XPLMGetDatai(gMcpApAltDataRef);
    gMcpVsButton = XPLMGetDatai(gMcpApVsDataRef);
    gMcpAprButton = XPLMGetDatai(gMcpApAprDataRef);
    gMcpRevButton = XPLMGetDatai(gMcpApRevDataRef);
}

void *mcpRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	uint32_t buttons = 0;
	int inReportBytesCount = 0;

	mcp_init();

	gPtrThreadData = (struct mcp_thread_data *) ptr_thread_data;

	int result = mcp_panel_open();
	if (result < 0) {
		XPLMDebugString("-> CP: mcp_controller.mcpRun: shutdown thread.\n");
		pthread_exit(NULL);
		return 0;
	}
	XPLMDebugString("-> CP: mcp_controller.mcp_run: panel opened.\n");

	last_mainloop_idle = sys_time_clock_get_time_usec();
	// while stop == 0 calculate position.
	while (gPtrThreadData->stop == 0) {
		long loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// Read panel for new messages. CRITICAL FAST 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(50000, COUNTER7, loop_start_time)) {
			// read/write board
			counter++;
			inReportBytesCount = mcp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 8) {
					sprintf(tmp, "-> CP: mcp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				} else {
//			    sprintf(tmp, "-> CP: mcp_controller.run: msg %d: %#0x,%#0x,%#0x\n", counter, buf[2], buf[1], buf[0]);
//				XPLMDebugString(tmp);
					counter2++;
					uint32_t msg = 0;
					msg += buf[4] << 24;
					msg += buf[3] << 16;
					msg += buf[2] << 8;
					msg += buf[1];
					mcp_process_knob(msg);
					msg += buf[7] << 24;
					msg += buf[6] << 16;
					msg += buf[5] << 8;
					msg += buf[4];
					mcp_process(msg);
				}
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// Update Panel. CRITICAL 2 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(500000, COUNTER8, loop_start_time)) {
			// Update local DataRefs.
			mcp_update_datarefs();
			// update Panel.
			inReportBytesCount = mcp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 8) {
					sprintf(tmp, "-> CP: mcp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				} else {
					uint32_t msg = 0;
					msg += buf[4] << 24;
					msg += buf[3] << 16;
					msg += buf[2] << 8;
					msg += buf[1];
					mcp_process(msg);
					msg += buf[7] << 24;
					msg += buf[6] << 16;
					msg += buf[5] << 8;
					msg += buf[4];
					mcp_process_knob(msg);
				}
			}
			uint32_t pos_negativ = MCP_LED_PLUS_SIGN;
			buttons = 0x00;
			if (gMcpApButton) {
				buttons |= 0x01;
			}
			if (gMcpHdgButton) {
				buttons |= 0x02;
			}
			if (gMcpNavButton) {
				buttons |= 0x04;
			}
			if (gMcpIasButton) {
				buttons |= 0x08;
			}
			if (gMcpAltButton) {
				buttons |= 0x10;
			}
			if (gMcpVsButton) {
				buttons |= 0x20;
			}
			if (gMcpAprButton) {
				buttons |= 0x40;
			}
			if (gMcpRevButton) {
				buttons |= 0x80;
			}
			mcp_led_update(tmp1, tmp2, pos_negativ, buttons, writeBuf);
			//mcp_panel_write(writeBuf);
			mcp_update_display();
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= MAX_DELAY_TIME) {
			XPLMDebugString("-> CP: mcp_controller.run: CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
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
	mcp_panel_close();
	pthread_exit(NULL);
	return 0;
}

