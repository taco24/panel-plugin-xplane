#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
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
    MCP_AP_FD_ON_CMD_MSG,
    MCP_AP_AUTOTHROTTLE_ON_CMD_MSG,
    MCP_AP_AUTOTHROTTLE_OFF_CMD_MSG,
    MCP_AP_HEADING_CMD_MSG,
    MCP_AP_NAV_CMD_MSG,
    MCP_AP_VNAV_CMD_MSG,
    MCP_AP_LVL_CHANGE_CMD_MSG,
    MCP_AP_IAS_CMD_MSG,
    MCP_AP_ALT_CMD_MSG,
    MCP_AP_VS_CMD_MSG,
    MCP_AP_APR_CMD_MSG,
    MCP_AP_REV_CMD_MSG,
    MCP_AP_AIRSPEED_SYNC_CMD_MSG,
    MCP_AP_HEADING_SYNC_CMD_MSG,
    MCP_AP_ALTITUDE_SYNC_CMD_MSG
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
static char tmp[100];

/* MULTI PANEL Command Refs */
XPLMCommandRef gMcpAltDnCmdRef = NULL;
XPLMCommandRef gMcpAltUpCmdRef = NULL;
XPLMCommandRef gMcpVrtclSpdDnCmdRef = NULL;
XPLMCommandRef gMcpVrtclSpdUpCmdRef = NULL;
XPLMCommandRef gMcpIASDnCmdRef = NULL;
XPLMCommandRef gMcpIASUpCmdRef = NULL;
XPLMCommandRef gMcpHdgDnCmdRef = NULL;
XPLMCommandRef gMcpHdgUpCmdRef = NULL;
XPLMCommandRef gMcpObsHsiDnCmdRef = NULL;
XPLMCommandRef gMcpObsHsiUpCmdRef = NULL;

XPLMCommandRef gMcpApFlightDirectorUpCmdRef = NULL;
XPLMCommandRef gMcpApFlightDirectorDnCmdRef = NULL;
XPLMCommandRef gMcpApFlightDirectorOffCmdRef = NULL;
XPLMCommandRef gMcpApFlightDirectorOnCmdRef = NULL;
XPLMCommandRef gMcpApAutoThrottleOnCmdRef = NULL;
XPLMCommandRef gMcpApAutoThrottleOffCmdRef = NULL;
XPLMCommandRef gMcpApAutopilotOnCmdRef = NULL;
XPLMCommandRef gMcpApAutopilotOffCmdRef = NULL;
XPLMCommandRef gMcpApHeadingCmdRef = NULL;
XPLMCommandRef gMcpApNAVCmdRef = NULL;
XPLMCommandRef gMcpApVNAVCmdRef = NULL;
XPLMCommandRef gMcpApIASLvlChgCmdRef = NULL;
XPLMCommandRef gMcpApAltCmdRef = NULL;
XPLMCommandRef gMcpApVsCmdRef = NULL;
XPLMCommandRef gMcpApAprCmdRef = NULL;
XPLMCommandRef gMcpApRevBackCourseCmdRef = NULL;
XPLMCommandRef gMcpApAirspeedSyncCmdRef = NULL;
XPLMCommandRef gMcpApHeadingSyncCmdRef = NULL;
XPLMCommandRef gMcpApAltitudeSyncCmdRef = NULL;


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
XPLMDataRef gMcpArspdIsMachDataRef = NULL;

XPLMDataRef gMcpAltDataRef = NULL;
XPLMDataRef gMcpVSDataRef = NULL;
XPLMDataRef gMcpIASDataRef = NULL;
XPLMDataRef gMcpHDGDataRef = NULL;
XPLMDataRef gMcpCRSDataRef = NULL;
static int gMcpAlt = 0;
static int gMcpVS = 0;
static int gMcpIAS = 0;
static int gMcpIASIsMach = 0;
static int gMcpVSSign = 0;
static int gMcpHDG = 0;
static int gMcpCRS = 0;

XPLMDataRef gMcpAvionicsPowerOnDataRef = NULL;
XPLMDataRef gMcpBatteryOnDataRef = NULL;
XPLMDataRef gMcpGeneratorArrayOnDataRef = NULL;
XPLMDataRef gMcpApAutopilotStateDataRef = NULL;
XPLMDataRef gMcpApAutopilotModeDataRef = NULL;
XPLMDataRef gMcpApMasterDataRef = NULL;
XPLMDataRef gMcpApMasterSourceDataRef = NULL;
XPLMDataRef gMcpApFlightDirectorDataRef = NULL;
XPLMDataRef gMcpApHdgDataRef = NULL;
XPLMDataRef gMcpApNavDataRef = NULL;
XPLMDataRef gMcpApVNavArmedDataRef = NULL;
XPLMDataRef gMcpApVNavDataRef = NULL;
XPLMDataRef gMcpApIASDataRef = NULL;
XPLMDataRef gMcpApAltDataRef = NULL;
XPLMDataRef gMcpApVsDataRef = NULL;
XPLMDataRef gMcpApAprDataRef = NULL;
XPLMDataRef gMcpApRevDataRef = NULL;
XPLMDataRef gMcpApAutoThrottleDataRef = NULL;
XPLMDataRef gMcpApGlideSlopeDataRef = NULL;
XPLMDataRef gMcpApTogaDataRef = NULL;
XPLMDataRef gMcpApTogaLateralDataRef = NULL;

static uint32_t gMcpAvionicsPowerOn = 0;
static uint32_t gMcpBatteryOn = 0;
static uint32_t gMcpAutopilotMode = 0;
static uint32_t gMcpApState = 0;
static uint32_t gMcpApSourceState = 0;
static uint32_t gMcpApFlightDirectorState = 0;
static uint32_t gMcpHdgState = 0;
static uint32_t gMcpNavState = 0;
static uint32_t gMcpVNavState = 0;
static uint32_t gMcpVNavArmed = 0;
static uint32_t gMcpGlideSlopeState = 0;
static uint32_t gMcpIasState = 0;
static uint32_t gMcpAltHoldState = 0;
static uint32_t gMcpVsState = 0;
static uint32_t gMcpAprState = 0;
static uint32_t gMcpRevState = 0;
static uint32_t gMcpAutoThrottleState = 0;
static uint32_t gMcpTogaState = 0;
static uint32_t gMcpTogaLateralState = 0;
static uint32_t gMcpAutopilotState = 0;
static uint32_t gMcpReadButtonAPState = 0;
static uint32_t gMcpReadButtonFDLState = 0;
static uint32_t gMcpReadButtonFDRState = 0;
static uint32_t gMcpReadButtonATState = 0;
static unsigned char leds1Prev = 0x00;
static unsigned char leds2Prev = 0x00;
static unsigned char leds3Prev = 0x00;
static unsigned char leds4Prev = 0x00;
static uint32_t toggleCRS = 0;
static uint32_t gMcpGeneratorArrayOn[8];


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
		    //gMcpAltNew = (int) XPLMGetDataf(gMcpAltHoldFtDataRef);
			break;
		case MCP_VS_UP_CMD_MSG:
		case MCP_VS_DN_CMD_MSG:
			//gMcpVSNew = (int) XPLMGetDataf(gMcpVrtVelDataRef);
			break;
		case MCP_IAS_UP_CMD_MSG:
		case MCP_IAS_DN_CMD_MSG:
			//gMcpIAS = (int) XPLMGetDataf(gMcpIASDataRef);
			break;
		case MCP_HDG_UP_CMD_MSG:
		case MCP_HDG_DN_CMD_MSG:
			//gMcpHDG = (int) XPLMGetDataf(gMcpHdgMagDataRef);
			break;
		case MCP_CRS_UP_CMD_MSG:
		case MCP_CRS_DN_CMD_MSG:
			//gMcpCRS = (int) XPLMGetDataf(gMcpHsiSrcSelPltDataRef);
			break;
		case MCP_AP_FD_UP_CMD_MSG:
		case MCP_AP_FD_DN_CMD_MSG:
		case MCP_AP_FD_OFF_CMD_MSG:
		case MCP_AP_FD_ON_CMD_MSG:
			gMcpApFlightDirectorState = XPLMGetDatai(gMcpApFlightDirectorDataRef);
			break;
		case MCP_AP_HEADING_CMD_MSG:
			gMcpHdgState = XPLMGetDatai(gMcpApHdgDataRef);
			break;
		case MCP_AP_NAV_CMD_MSG:
			gMcpNavState = XPLMGetDatai(gMcpApNavDataRef);
			break;
		case MCP_AP_VNAV_CMD_MSG:
			gMcpVNavState = XPLMGetDatai(gMcpApVNavDataRef);
			break;
		case MCP_AP_IAS_CMD_MSG:
			gMcpIasState = XPLMGetDatai(gMcpApIASDataRef);
			break;
		case MCP_AP_ALT_CMD_MSG:
			gMcpAltHoldState = XPLMGetDatai(gMcpApAltDataRef);
			break;
		case MCP_AP_VS_CMD_MSG:
			gMcpVsState = XPLMGetDatai(gMcpApVsDataRef);
			break;
		case MCP_AP_APR_CMD_MSG:
			gMcpAprState = XPLMGetDatai(gMcpApAprDataRef);
			break;
		case MCP_AP_REV_CMD_MSG:
			gMcpRevState = XPLMGetDatai(gMcpApRevDataRef);
			break;
		default:
			break;
 }
 return status;
}

/*
--------------------------------
LEDs:
--------------------------------
0F 00 00 00 00 -> BLANK ALL
0F 00 01 00 00 -> SPEED
0F 00 02 00 00 -> LVL CHG
0F 00 04 00 00 -> HDG SEL
0F 00 08 00 00 -> APP
0F 00 10 00 00 -> ALTHLD
0F 00 20 00 00 -> V/S
0F 00 40 00 00 ->
0F 00 80 00 00 -> F/D (R)
0F 00 00 01 00 ->
0F 00 00 02 00 -> CWS (L) (Control Wheel Steering)
0F 00 00 04 00 -> CWS (R)
0F 00 00 08 00 ->
0F 00 00 10 00 ->
0F 00 00 20 00 ->
0F 00 00 40 00 -> F/D (L)
0F 00 00 80 00 -> N1
0F 00 00 00 01 -> VNAV
0F 00 00 00 02 -> LNAV
0F 00 00 00 04 -> ACMD
0F 00 00 00 08 -> BCMD
0F 00 00 00 10 -> AutoThrottle/ARM
0F 00 00 00 20 ->
0F 00 00 00 40 ->
0F 00 00 00 80 -> VOR LOC
0F 00 FF FF FF -> ALL LEDS

http://www.airliners.net/aviation-forums/tech_ops/read.main/17539/
N1: The Autothrottle will control the speed of the
    aircraft to the N1 limit set by the Power Mgmt Computer.
LVL CHG: Tricky one for me. Level Change controls the
    aircrafts thrust thru the A/T and speed thru the elevators.
    The altitude selected on the MCP is what Level Change is
    shooting for.
Speed: Aircraft speed is controlled by A/T and set to
    maintain what ever speed is dialed into MCP speed window.

*/
inline void mcp_update_leds() {
	unsigned char leds1 = 0x00;
	unsigned char leds2 = 0x00;
	unsigned char leds3 = 0x00;
	unsigned char leds4 = 0x00;
	// Testing
	//leds4 = 0x80;

	if (gMcpAutopilotMode == 2) {
		// Autopilot
		leds4 |= 0x04;
		leds4 |= 0x08;
		// Flight Director
		leds3 |= 0x40;
		leds2 |= 0x80;
	} else if (gMcpAutopilotMode == 1) {
		// Flight Director
		leds3 |= 0x40;
		leds2 |= 0x80;
	}
	if (gMcpHdgState) {
		leds2 |= 0x04;
	}
	if ((gMcpAutopilotState) & 0x4000) {
		// Altitude Hold Engaged
		leds2 |= 0x10;
	}
	if ((gMcpAutopilotState) & 0x1000) {
		// VNAV Armed
		leds4 |= 0x01;
	}
	if ((gMcpAutopilotState) & 0x0400) {
		// 0x0400 Glideslope Armed (APProach)
		leds2 |= 0x08;
	}
	if ((gMcpAutopilotState) & 0x0300) {
		// 0x0100 HNAV Armed
		// 0x0200 LOCalizer
		leds4 |= 0x02;
		leds4 |= 0x80;
	}
	if ((gMcpAutopilotState) & 0x0040) {
		// 0x0040 Flight Level Change Engage
		leds2 |= 0x02;
	}
	if ((gMcpAutopilotState) & 0x0020) {
		// 0x0020 Altitude Hold Arm
		leds2 |= 0x10;
	}
	if ((gMcpAutopilotState) & 0x0010) {
		// 0x0010 VVI Climb Engage
		leds2 |= 0x20;
	}
	if ((gMcpAutopilotState) & 0x0008) {
		// 0x0008 Airspeed Hold With Pitch Engage
		leds2 |= 0x01;
	}
	if ((gMcpAutopilotState) & 0x0002) {
		// 0x0002 Heading Hold Engage
		leds2 |= 0x04;
	}
	if ((gMcpAutopilotState) & 0x0001) {
		// 0x0001 Autothrottle Engage
		leds4 |= 0x10;
	}

/*	char Buffer[256];
	sprintf(Buffer,"%d,%d,%d: %d, %d, %d, -%d, %d, %d, -%d, %d, %d, -%d, %d, %d, -%d, %#0x, %d\n",
			gMcpApState, gMcpApSourceState, gMcpApFlightDirectorState,
			gMcpHdgState, gMcpNavState,	gMcpVNavState,
			gMcpIasState, gMcpAltHoldState,	gMcpVsState,
			gMcpAprState, gMcpRevState,	gMcpAutoThrottleState,
			gMcpTogaState, gMcpTogaLateralState, gMcpGlideSlopeState,
			(gMcpAutopilotState), gMcpVNavArmed);
	XPLMDebugString(Buffer);
*/	if (gMcpVNavState) {
		leds4 |= 0x01;
	}
	if (gMcpNavState) {
		// HNAV armed: VOR LOCalizer
		leds4 |= 0x02;
		leds4 |= 0x80;
	}
	if (gMcpIasState == 1) {
		// Speed Hold
	} else if (gMcpIasState == 2) {
		// LVL Change
		leds2 |= 0x02;
	}
	if (gMcpAltHoldState) {
		leds2 |= 0x10;
	}
	if (gMcpVsState) {
		leds2 |= 0x20;
	}
	if (gMcpAprState) {
		leds2 |= 0x08;
	}
	if (gMcpRevState) {
		leds2 |= 0x08;
	}
	if (gMcpAutoThrottleState) {
		leds4 |= 0x10;
	}
	if (leds1 == 0x00
			&& leds2 == 0x00
			&& leds3 == 0x00
			&& leds4 == 0x00) {
		// Prevent all LEDs off
		leds2 = 0x10;
		leds2Prev = leds2;
	}
	// Avionics Power on?
	if (gMcpAvionicsPowerOn && (gMcpBatteryOn || gMcpGeneratorArrayOn[0])) {
		mcp_leds[1] = leds1;
		mcp_leds[2] = leds2;
		mcp_leds[3] = leds3;
		mcp_leds[4] = leds4;
		if (!(leds1 == leds1Prev
				&& leds2 == leds2Prev
				&& leds3 == leds3Prev
				&& leds4 == leds4Prev)) {
			leds1Prev = leds1;
			leds2Prev = leds2;
			leds3Prev = leds3;
			leds4Prev = leds4;
			mcp_panel_write(mcp_leds);
		}
	} else {
		mcp_leds[1] = 0;
		mcp_leds[2] = 0;
		mcp_leds[3] = 0;
		mcp_leds[4] = 0;
		leds1Prev = 0;
		leds2Prev = 0;
		leds3Prev = 0;
		leds4Prev = 0;
		mcp_panel_write(mcp_leds);
	}
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
		toggleCRS++;
		mcp_course_left[1] = mcp_get_digit(gMcpCRS / 100);
		mcp_course_left[2] = mcp_get_digit(gMcpCRS % 100 / 10);
		mcp_course_left[3] = mcp_get_digit(gMcpCRS % 10);
		mcp_panel_write(mcp_course_left);
	} else if (toggleCRS) {
		gMcpCRS = round(XPLMGetDataf(gMcpHsiObsDegMagPltDataRef));
		mcp_course_right[1] = mcp_get_digit(gMcpCRS / 100);
		mcp_course_right[2] = mcp_get_digit(gMcpCRS % 100 / 10);
		mcp_course_right[3] = mcp_get_digit(gMcpCRS % 10);
		mcp_panel_write(mcp_course_right);
		toggleCRS = 0;
	}
	if (gMcpAlt != round(XPLMGetDataf(gMcpAltHoldFtDataRef))) {
		gMcpAlt = round(XPLMGetDataf(gMcpAltHoldFtDataRef));
		mcp_altitude[1] = mcp_get_digit(gMcpAlt / 10000);
		mcp_altitude[2] = mcp_get_digit(gMcpAlt % 10000 / 1000);
		mcp_altitude[3] = mcp_get_digit(gMcpAlt % 1000 / 100);
		mcp_altitude[4] = mcp_get_digit(gMcpAlt % 100 / 10);
		mcp_altitude[5] = mcp_get_digit(gMcpAlt % 10);
		mcp_panel_write(mcp_altitude);
	}
	if (gMcpHDG != round(XPLMGetDataf(gMcpHdgMagDataRef))) {
		gMcpHDG = round(XPLMGetDataf(gMcpHdgMagDataRef));
		mcp_heading[1] = mcp_get_digit(gMcpHDG / 100);
		mcp_heading[2] = mcp_get_digit(gMcpHDG % 100 / 10);
		mcp_heading[3] = mcp_get_digit(gMcpHDG % 10);
		mcp_panel_write(mcp_heading);
	}
	if (gMcpVS != round(XPLMGetDataf(gMcpVSDataRef))) {
		gMcpVS = round(XPLMGetDataf(gMcpVSDataRef));
		int value = gMcpVS;
		if (gMcpVS < 0) {
			gMcpVSSign = 0x40;
			value = value * -1;
		} else {
			gMcpVSSign = 0x00;
		}
		mcp_vert_speed[1] = gMcpVSSign;
		mcp_vert_speed[2] = mcp_get_digit(value % 10000 / 1000);
		mcp_vert_speed[3] = mcp_get_digit(value % 1000 / 100);
		mcp_vert_speed[4] = mcp_get_digit(value % 100 / 10);
		mcp_vert_speed[5] = mcp_get_digit(value % 10);
		mcp_panel_write(mcp_vert_speed);
	}
    if (gMcpIASIsMach) {
    	if (gMcpIAS != round(XPLMGetDataf(gMcpIASDataRef) * 100)) {
    		gMcpIAS = round(XPLMGetDataf(gMcpIASDataRef) * 100);
    		mcp_ias_mach[1] = 0x00;
    		mcp_ias_mach[2] = 0x00;
    		mcp_ias_mach[3] = mcp_get_digit(gMcpIAS / 100) | 0x80; // decimal point
    		mcp_ias_mach[4] = mcp_get_digit(gMcpIAS % 100 / 10);
    		mcp_ias_mach[5] = mcp_get_digit(gMcpIAS % 10);
    		mcp_panel_write(mcp_ias_mach);
    	}
    } else {
    	if (gMcpIAS != round(XPLMGetDataf(gMcpIASDataRef))) {
    		gMcpIAS = round(XPLMGetDataf(gMcpIASDataRef));
    		mcp_ias_mach[1] = 0x00;
    		mcp_ias_mach[2] = 0x00;
    		mcp_ias_mach[3] = mcp_get_digit(gMcpIAS % 1000 / 100);
    		mcp_ias_mach[4] = mcp_get_digit(gMcpIAS % 100 / 10);
    		mcp_ias_mach[5] = mcp_get_digit(gMcpIAS % 10);
    		mcp_panel_write(mcp_ias_mach);
    	}
    }

}

int mcp_process(uint32_t msg) {
//    sprintf(tmp, "-> CP: mcp_controller.mcp_process: msg: %d\n", msg);
//	XPLMDebugString(tmp);
	int res = 0;
    uint32_t readPushButtons = msg & MCP_READ_PUSH_BUTTONS_MASK;
    uint32_t readButtons = msg & MCP_READ_BUTTONS_MASK;
    uint32_t readAP = readButtons & MCP_READ_BTN_DISENGAGE;
    uint32_t readFDL = readButtons & MCP_READ_BTN_F_D;
    uint32_t readFDR = readButtons & MCP_READ_BTN_F_D_RIGHT;
    uint32_t readAT = readButtons & MCP_READ_BTN_A_T_ARM;

//    char tmp3[100];
//	sprintf(tmp3, "-> read %#0x, %#0x\n", readPushButtons, readButtons);
//	XPLMDebugString(tmp3);

    if (readPushButtons) {
    	if (readPushButtons == MCP_READ_BTN_HEADING) {
    		XPLMCommandOnce(gMcpApHeadingSyncCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_IAS_MACH) {
    		XPLMCommandOnce(gMcpApAirspeedSyncCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_ALTITUDE) {
    		XPLMCommandOnce(gMcpApAltitudeSyncCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_HDG_SEL) {
    		XPLMCommandOnce(gMcpApHeadingCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_LNAV) {
    		XPLMCommandKeyStroke(xplm_key_otto_hnav);
    	} else if (readPushButtons == MCP_READ_BTN_VNAV) {
    		XPLMCommandKeyStroke(xplm_key_otto_vnav);
        } else if (readPushButtons == MCP_READ_BTN_V_S) {
        	XPLMCommandOnce(gMcpApVsCmdRef);
       	} else if (readPushButtons == MCP_READ_BTN_C_O) {
       		if (gMcpIASIsMach) {
       			XPLMSetDatai(gMcpArspdIsMachDataRef, 0);
       			XPLMSetDataf(gMcpArspdDataRef, ((float) gMcpIAS) * 6.0);
       		} else {
       			XPLMSetDatai(gMcpArspdIsMachDataRef, 1);
       			XPLMSetDataf(gMcpArspdDataRef, ((float) gMcpIAS) / 600.0);
       		}
    	} else if (readPushButtons == MCP_READ_BTN_APP) {
    		XPLMCommandOnce(gMcpApAprCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_VOR_LOC) {
    		XPLMCommandOnce(gMcpApNAVCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_ALT_HLD) {
    		XPLMCommandOnce(gMcpApAltCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_LVL_CHANGE) {
    		XPLMCommandOnce(gMcpApIASLvlChgCmdRef);
    	} else if (readPushButtons == MCP_READ_BTN_ACMD || readPushButtons == MCP_READ_BTN_BCMD) {
    		if (gMcpAutopilotMode == 2) {
    			//XPLMSetDatai(gMcpApAutopilotModeDataRef, 0);
    			XPLMCommandOnce(gMcpApFlightDirectorOffCmdRef);
    		} else {
    			XPLMCommandOnce(gMcpApFlightDirectorOnCmdRef);
    		}
     	}
    }

	if (gMcpReadButtonAPState != readAP) {
		gMcpReadButtonAPState = readAP;
		// DISENGAGE
		if (readAP) {
			// disengage AP
			XPLMCommandOnce(gMcpApFlightDirectorOffCmdRef);
		}
	}
	if (gMcpReadButtonFDLState != readFDL) {
		gMcpReadButtonFDLState = readFDL;
		// Left FD Changed
		if (readFDL) {
			XPLMSetDatai(gMcpApAutopilotModeDataRef, 1);
		} else {
			XPLMCommandOnce(gMcpApFlightDirectorOffCmdRef);
		}
	}
	if (gMcpReadButtonFDRState != readFDR) {
		gMcpReadButtonFDRState = readFDR;
		// Right FD Changed
		if (readFDR) {
			XPLMSetDatai(gMcpApAutopilotModeDataRef, 1);
		} else {
			XPLMCommandOnce(gMcpApFlightDirectorOffCmdRef);
		}
	}
	if (gMcpReadButtonATState != readAT) {
		gMcpReadButtonATState = readAT;
		// Autothrottle Changed
		if (readAT) {
			//XPLMCommandOnce(gMcpApAutoThrottleOnCmdRef);
			if (!((gMcpAutopilotState) & 0x0001)) {
				// 0x0001 Autothrottle Engage
				XPLMCommandKeyStroke(xplm_key_otto_atr);
			}
		} else {
			//XPLMCommandOnce(gMcpApAutoThrottleOffCmdRef);
			if ((gMcpAutopilotState) & 0x0001) {
				// 0x0001 Autothrottle Disengage
				XPLMCommandKeyStroke(xplm_key_otto_atr);
			}
		}
	}


	return res;
}

int mcp_process_knob(uint32_t msg) {
//    sprintf(tmp, "-> CP: mcp_controller.mcp_process: msg: %d\n", msg);
//	XPLMDebugString(tmp);
	int res = 0;
    uint32_t readCourseLeft = msg & MCP_READ_KNOB_COURSE_L_MASK;
    uint32_t readIASMach = msg & MCP_READ_KNOB_IAS_MACH_MASK;
    uint32_t readHeading = msg & MCP_READ_KNOB_HEADING_MASK;
    uint32_t readAltitute = msg & MCP_READ_KNOB_ALTITUDE_MASK;
    uint32_t readVertSpeed = msg & MCP_READ_KNOB_VERTSPEED_MASK;
    uint32_t readCourseRight = msg & MCP_READ_KNOB_COURSE_R_MASK;

    if (readCourseLeft) {
    	int c1 = msg >> 4;
    	int i;
//    	sprintf(tmp, "mcp_process_knob: c1: %d\n", c1);
//    	XPLMDebugString(tmp);
    	if (c1 < 8) {
    		if (c1 > 2) {
    			c1 *= 3;
    		}
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpObsHsiUpCmdRef);
    		}
    	} else {
    		c1 = (17 - c1);
    		if (c1 > 2) {
    			c1 *= 3;
    		}
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpObsHsiDnCmdRef);
    		}
    	}
    }
    if (readCourseRight) {
    	int c1 = msg >> 16;
    	int i;
    	if (c1 < 8) {
    		if (c1 > 2) {
    			c1 *= 3;
    		}
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpObsHsiUpCmdRef);
    		}
    	} else {
    		c1 = (17 - c1);
    		if (c1 > 2) {
    			c1 *= 3;
    		}
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpObsHsiDnCmdRef);
    		}
    	}
    }
    if (readIASMach) {
    	if (gMcpIASIsMach) {
    		float iasMach = XPLMGetDataf(gMcpIASDataRef);
        	int c1 = msg >> 12;
        	int i;
        	if (c1 < 8) {
        		for (i = 0; i < c1; i++) {
        			iasMach += 0.01;
        		}
        	} else {
        		for (i = 0; i < (16 - c1); i++) {
        			iasMach -= 0.01;
        		}
        	}
    		XPLMSetDataf(gMcpIASDataRef, iasMach);
    	} else {
        	int c1 = msg >> 12;
        	int i;
        	if (c1 < 8) {
        		if (c1 > 2) {
        			c1 *= 3;
        		}
        		for (i = 0; i < c1; i++) {
        			XPLMCommandOnce(gMcpIASUpCmdRef);
        		}
        	} else {
        		c1 = (17 - c1);
        		if (c1 > 2) {
        			c1 *= 3;
        		}
        		for (i = 0; i < c1; i++) {
        			XPLMCommandOnce(gMcpIASDnCmdRef);
        		}
        	}
    	}
    }
    if (readHeading) {
    	int c1 = msg;
    	int i;
    	if (c1 < 8) {
    		if (c1 > 2) {
    			c1 *= 3;
    		}
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpHdgUpCmdRef);
    		}
    	} else {
    		c1 = (17 - c1);
    		if (c1 > 2) {
    			c1 *= 3;
    		}
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpHdgDnCmdRef);
    		}
    	}
    }
    if (readAltitute) {
    	int c1 = msg >> 20;
    	int i;
    	if (c1 < 8) {
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpAltUpCmdRef);
    		}
    	} else {
    		for (i = 0; i < (16 - c1); i++) {
    			XPLMCommandOnce(gMcpAltDnCmdRef);
    		}
    	}
    }
    if (readVertSpeed) {
    	int c1 = msg >> 8;
    	int i;
    	if (c1 < 8) {
    		for (i = 0; i < c1; i++) {
    			XPLMCommandOnce(gMcpVrtclSpdUpCmdRef);
    		}
    	} else {
    		for (i = 0; i < (16 - c1); i++) {
    			XPLMCommandOnce(gMcpVrtclSpdDnCmdRef);
    		}
    	}
    }


	return res;
}


void mcp_update_datarefs() {
	gMcpAvionicsPowerOn = XPLMGetDatai(gMcpAvionicsPowerOnDataRef);
	gMcpBatteryOn = XPLMGetDatai(gMcpBatteryOnDataRef);
	XPLMSetDatavi(gMcpGeneratorArrayOnDataRef, gMcpGeneratorArrayOn, 0, 8);
	gMcpAutopilotMode = XPLMGetDatai(gMcpApAutopilotModeDataRef);
    gMcpAutopilotState = XPLMGetDatai(gMcpApAutopilotStateDataRef);
    gMcpApState = XPLMGetDatai(gMcpApMasterDataRef);
    gMcpApSourceState = XPLMGetDatai(gMcpApMasterSourceDataRef);
    gMcpApFlightDirectorState = XPLMGetDatai(gMcpApFlightDirectorDataRef);
    gMcpHdgState = XPLMGetDatai(gMcpApHdgDataRef);
    gMcpNavState = XPLMGetDatai(gMcpApNavDataRef);
    gMcpIasState = XPLMGetDatai(gMcpApIASDataRef);
    gMcpAltHoldState = XPLMGetDatai(gMcpApAltDataRef);
    gMcpVsState = XPLMGetDatai(gMcpApVsDataRef);
    gMcpAprState = XPLMGetDatai(gMcpApAprDataRef);
    gMcpRevState = XPLMGetDatai(gMcpApRevDataRef);
    gMcpVNavState = XPLMGetDatai(gMcpApVNavDataRef);
    gMcpVNavArmed = XPLMGetDatai(gMcpApVNavArmedDataRef);
    gMcpGlideSlopeState = XPLMGetDatai(gMcpApGlideSlopeDataRef);
    gMcpAutoThrottleState = XPLMGetDatai(gMcpApAutoThrottleDataRef);
    gMcpIASIsMach = XPLMGetDatai(gMcpArspdIsMachDataRef);
    gMcpTogaState = XPLMGetDatai(gMcpApTogaDataRef);
    gMcpTogaLateralState = XPLMGetDatai(gMcpApTogaLateralDataRef);

}

void mcp_init() {
//	XPLMDebugString("-> CP: mcp_controller.mcp_init.\n");

    gMcpAltDnCmdRef          = XPLMFindCommand(sMCP_ALTITUDE_DOWN_CR);
    gMcpAltUpCmdRef          = XPLMFindCommand(sMCP_ALTITUDE_UP_CR);
    gMcpVrtclSpdDnCmdRef     = XPLMFindCommand(sMCP_VERTICAL_SPEED_DOWN_CR);
    gMcpVrtclSpdUpCmdRef     = XPLMFindCommand(sMCP_VERTICAL_SPEED_UP_CR);
    gMcpIASDnCmdRef           = XPLMFindCommand(sMCP_AIRSPEED_DOWN_CR);
    gMcpIASUpCmdRef           = XPLMFindCommand(sMCP_AIRSPEED_UP_CR);
    gMcpHdgDnCmdRef          = XPLMFindCommand(sMCP_HEADING_DOWN_CR);
    gMcpHdgUpCmdRef          = XPLMFindCommand(sMCP_HEADING_UP_CR);
    gMcpObsHsiDnCmdRef       = XPLMFindCommand(sMCP_OBS_HSI_DOWN_CR);
    gMcpObsHsiUpCmdRef       = XPLMFindCommand(sMCP_OBS_HSI_UP_CR);

    XPLMRegisterCommandHandler(gMcpAltDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_ALT_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpAltUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_ALT_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpVrtclSpdDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_VS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpVrtclSpdUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_VS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpIASDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_IAS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpIASUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_IAS_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpHdgDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_HDG_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpHdgUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_HDG_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpObsHsiDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_CRS_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpObsHsiUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_CRS_DN_CMD_MSG);

    gMcpApFlightDirectorUpCmdRef  = XPLMFindCommand(sMCP_FDIR_SERVOS_UP_ONE_CR);
    gMcpApFlightDirectorDnCmdRef  = XPLMFindCommand(sMCP_FDIR_SERVOS_DOWN_ONE_CR);
    gMcpApFlightDirectorOffCmdRef = XPLMFindCommand(sMCP_SERVOS_AND_FLIGHT_DIR_OFF_CR);
    gMcpApFlightDirectorOnCmdRef  = XPLMFindCommand(sMCP_SERVOS_AND_FLIGHT_DIR_ON_CR);
    gMcpApAutoThrottleOnCmdRef    = XPLMFindCommand(sMCP_AP_AUTOTHROTTLE_ON_CR);
    gMcpApAutoThrottleOffCmdRef   = XPLMFindCommand(sMCP_AP_AUTOTHROTTLE_OFF_CR);
    gMcpApHeadingCmdRef           = XPLMFindCommand(sMCP_AP_HEADING_CR);
    gMcpApNAVCmdRef               = XPLMFindCommand(sMCP_AP_NAV_CR);
    gMcpApVNAVCmdRef              = XPLMFindCommand(sMCP_AP_VNAV_CR);
    gMcpApIASLvlChgCmdRef         = XPLMFindCommand(sMCP_AP_LEVEL_CHANGE_CR);
    gMcpApAltCmdRef               = XPLMFindCommand(sMCP_AP_ALTITUDE_HOLD_CR);
    gMcpApVsCmdRef                = XPLMFindCommand(sMCP_AP_VERTICAL_SPEED_CR);
    gMcpApAprCmdRef               = XPLMFindCommand(sMCP_AP_APPROACH_CR);
    gMcpApRevBackCourseCmdRef     = XPLMFindCommand(sMCP_AP_BACK_COURSE_CR);
    gMcpApAirspeedSyncCmdRef      = XPLMFindCommand(sMCP_AP_AIRSPEED_SYNC_CR);
    gMcpApHeadingSyncCmdRef       = XPLMFindCommand(sMCP_AP_HEADING_SYNC_CR);
    gMcpApAltitudeSyncCmdRef      = XPLMFindCommand(sMCP_AP_ALTITUDE_SYNC_CR);

    XPLMRegisterCommandHandler(gMcpApFlightDirectorUpCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_UP_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApFlightDirectorDnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_DN_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApFlightDirectorOffCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_OFF_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApFlightDirectorOnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_FD_ON_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAutoThrottleOnCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_AUTOTHROTTLE_ON_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAutoThrottleOffCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_AUTOTHROTTLE_OFF_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApHeadingCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_HEADING_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApNAVCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_NAV_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApVNAVCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_VNAV_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApIASLvlChgCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_IAS_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAltCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_ALT_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApVsCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_VS_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAprCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_APR_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApRevBackCourseCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_REV_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAirspeedSyncCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_AIRSPEED_SYNC_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApHeadingSyncCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_HEADING_SYNC_CMD_MSG);
    XPLMRegisterCommandHandler(gMcpApAltitudeSyncCmdRef, MainControlPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) MCP_AP_ALTITUDE_SYNC_CMD_MSG);

    gMcpAltHoldFtDataRef         = XPLMFindDataRef(sMCP_ALTITUDE_DIAL_FT_DR);
    gMcpVrtVelDataRef            = XPLMFindDataRef(sMCP_VVI_DIAL_FPM_DR);
    gMcpVSDataRef                = XPLMFindDataRef(sMCP_VVI_DIAL_FPM_DR);
    gMcpArspdDataRef             = XPLMFindDataRef(sMCP_AIRSPEED_DR);
    gMcpIASDataRef               = XPLMFindDataRef(sMCP_AIRSPEED_DR);
    gMcpHdgMagDataRef            = XPLMFindDataRef(sMCP_HSI_BEARING_DEG_MAG_PILOT_DR);
    gMcpHsiObsDegMagPltDataRef   = XPLMFindDataRef(sMCP_HSI_OBS_DEG_MAG_PILOT_DR);
    gMcpHsiSrcSelPltDataRef      = XPLMFindDataRef(sMCP_HSI_SOURCE_SELECT_PILOT_DR);
    gMcpNav1CrsDefMagPltDataRef  = XPLMFindDataRef(sMCP_NAV1_COURSE_DEG_MAG_PILOT_DR);
    gMcpNav2CrsDefMagPltDataRef  = XPLMFindDataRef(sMCP_NAV2_COURSE_DEG_MAG_PILOT_DR);
    gMcpGpsCourseDataRef         = XPLMFindDataRef(sMCP_GPS_COURSE_DR);
    gMcpArspdIsMachDataRef       = XPLMFindDataRef(sMCP_AP_AIRSPEED_IS_MACH_DR);

    gMcpAlt = round(XPLMGetDataf(gMcpAltHoldFtDataRef));
    gMcpVS = round(XPLMGetDataf(gMcpVSDataRef));
    gMcpIASIsMach = XPLMGetDatai(gMcpArspdIsMachDataRef);
    if (gMcpIASIsMach) {
        gMcpIAS = round(XPLMGetDataf(gMcpIASDataRef) * 100);
    } else {
        gMcpIAS = round(XPLMGetDataf(gMcpIASDataRef));
    }
    gMcpHDG = round(XPLMGetDataf(gMcpHdgMagDataRef));
    gMcpCRS = round(XPLMGetDataf(gMcpHsiObsDegMagPltDataRef));

    gMcpAvionicsPowerOnDataRef = XPLMFindDataRef(sMCP_AVIONICS_POWER_ON_DR);
    gMcpBatteryOnDataRef = XPLMFindDataRef(sMCP_BATTERY_ON_DR);
    gMcpGeneratorArrayOnDataRef = XPLMFindDataRef(sMCP_GENERATOR_ARRAY_ON_DR);
    gMcpApAutopilotModeDataRef = XPLMFindDataRef(sMCP_AUTOPILOT_MODE_DR);
    gMcpApAutopilotStateDataRef = XPLMFindDataRef(sMCP_AUTOPILOT_STATE_DR);
    gMcpApMasterDataRef = XPLMFindDataRef(sMCP_AP_AUTOPILOT_ON_DR);
    gMcpApMasterSourceDataRef = XPLMFindDataRef(sMCP_AP_AUTOPILOT_SOURCE_DR);
    gMcpApFlightDirectorDataRef = XPLMFindDataRef(sMCP_AP_FLIGHT_DIRECTOR_MODE_DR);
    gMcpApHdgDataRef = XPLMFindDataRef(sMCP_AP_HEADING_STATUS_DR);
    gMcpApNavDataRef = XPLMFindDataRef(sMCP_AP_HNAV_ARMED_DR);
    gMcpApIASDataRef = XPLMFindDataRef(sMCP_AP_SPEED_STATUS_DR);
    gMcpApAltDataRef = XPLMFindDataRef(sMCP_AP_ALTITUDE_HOLD_STATUS_DR);
    gMcpApVsDataRef = XPLMFindDataRef(sMCP_AP_VVI_STATUS_DR);
    gMcpApAprDataRef = XPLMFindDataRef(sMCP_AP_APPROACH_STATUS_DR);
    gMcpApRevDataRef = XPLMFindDataRef(sMCP_AP_BACKCOURSE_STATUS_DR);
    gMcpApVNavDataRef = XPLMFindDataRef(sMCP_AP_VNAV_STATUS_DR);
    gMcpApVNavArmedDataRef = XPLMFindDataRef(sMCP_AP_VNAV_ARMED_DR);
    gMcpApGlideSlopeDataRef = XPLMFindDataRef(sMCP_AP_GLIDE_SLOPE_STATUS_DR);
    gMcpApAutoThrottleDataRef = XPLMFindDataRef(sMCP_AP_AUTOTHROTTLE_ON_DR);
    gMcpApTogaDataRef = XPLMFindDataRef(sMCP_AP_TOGA_STATUS_DR);
    gMcpApTogaLateralDataRef = XPLMFindDataRef(sMCP_AP_TOGA_LATERAL_STATUS_DR);


	gMcpAvionicsPowerOn = XPLMGetDatai(gMcpAvionicsPowerOnDataRef);
	gMcpBatteryOn = XPLMGetDatai(gMcpBatteryOnDataRef);
	XPLMSetDatavi(gMcpGeneratorArrayOnDataRef, gMcpGeneratorArrayOn, 0, 8);
    gMcpAutopilotMode = XPLMGetDatai(gMcpApAutopilotModeDataRef);
    gMcpAutopilotState = XPLMGetDatai(gMcpApAutopilotStateDataRef);
    gMcpApState = XPLMGetDatai(gMcpApMasterDataRef);
    gMcpApSourceState = XPLMGetDatai(gMcpApMasterSourceDataRef);
    gMcpApFlightDirectorState = XPLMGetDatai(gMcpApFlightDirectorDataRef);
    gMcpHdgState = XPLMGetDatai(gMcpApHdgDataRef);
    gMcpNavState = XPLMGetDatai(gMcpApNavDataRef);
    gMcpVNavState = XPLMGetDatai(gMcpApVNavDataRef);
    gMcpVNavArmed = XPLMGetDatai(gMcpApVNavArmedDataRef);
    gMcpGlideSlopeState = XPLMGetDatai(gMcpApGlideSlopeDataRef);
    gMcpIasState = XPLMGetDatai(gMcpApIASDataRef);
    gMcpAltHoldState = XPLMGetDatai(gMcpApAltDataRef);
    gMcpVsState = XPLMGetDatai(gMcpApVsDataRef);
    gMcpAprState = XPLMGetDatai(gMcpApAprDataRef);
    gMcpRevState = XPLMGetDatai(gMcpApRevDataRef);
    gMcpAutoThrottleState = XPLMGetDatai(gMcpApAutoThrottleDataRef);
    gMcpTogaState = XPLMGetDatai(gMcpApTogaDataRef);
    gMcpTogaLateralState = XPLMGetDatai(gMcpApTogaLateralDataRef);

}

void *mcpRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	int inReportBytesCount = 0;

#if IBM
		Sleep(SLEEP_TIME * 4);
#else
		usleep(SLEEP_TIME * 4);
#endif

	mcp_init();

	gPtrThreadData = (struct mcp_thread_data *) ptr_thread_data;

	int result = mcp_panel_open();
	if (result < 0) {
		XPLMDebugString("-> CP: mcp_controller.mcpRun: shutdown thread.\n");
		gPtrThreadData->stop = 1;
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
					uint32_t msg1 = 0;
					uint32_t msg2 = 0;
					msg1 += buf[7] << 16;
					msg1 += buf[6] << 8;
					msg1 += buf[5];
					mcp_process(msg1);
					msg2 += buf[3] << 16;
					msg2 += buf[2] << 8;
					msg2 += buf[1];
					mcp_process_knob(msg2);

//						char Buffer[256];
//						sprintf(Buffer,"-> %#0x, %#0x\n",msg1, msg2);
//						XPLMDebugString(Buffer);
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
					uint32_t msg1 = 0;
					uint32_t msg2 = 0;
					msg1 += buf[7] << 16;
					msg1 += buf[6] << 8;
					msg1 += buf[5];
					mcp_process(msg1);
					msg2 += buf[3] << 16;
					msg2 += buf[2] << 8;
					msg2 += buf[1];
					mcp_process_knob(msg2);

//					char Buffer[256];
//					sprintf(Buffer,"-> %#0x, %#0x\n",msg1, msg2);
//					XPLMDebugString(Buffer);
				}
			}
			mcp_update_leds();
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
		Sleep(SLEEP_TIME * 4);
#else
		usleep(SLEEP_TIME * 4);
#endif
	}
	mcp_panel_close();
#if IBM
		Sleep(SLEEP_TIME * 4);
#else
		usleep(SLEEP_TIME * 4);
#endif
	pthread_exit(NULL);
	return 0;
}

