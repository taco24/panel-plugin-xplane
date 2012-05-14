#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "cb_controller.h"
#include "cb_driver.h"
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
enum CB_COMMANDS_MAP {
	CB_CMD_STDBY_COM1_FINE_DOWN,
	CB_CMD_STDBY_COM1_FINE_UP,
	CB_CMD_STDBY_COM1_COARSE_DOWN,
	CB_CMD_STDBY_COM1_COARSE_UP,
	CB_CMD_ACTV_COM1_FINE_DOWN,
	CB_CMD_ACTV_COM1_FINE_UP,
	CB_CMD_ACTV_COM1_COARSE_DOWN,
	CB_CMD_ACTV_COM1_COARSE_UP,
	CB_CMD_COM1_STANDBY_FLIP,

	CB_CMD_STDBY_COM2_FINE_DOWN,
	CB_CMD_STDBY_COM2_FINE_UP,
	CB_CMD_STDBY_COM2_COARSE_DOWN,
	CB_CMD_STDBY_COM2_COARSE_UP,
	CB_CMD_ACTV_COM2_FINE_DOWN,
	CB_CMD_ACTV_COM2_FINE_UP,
	CB_CMD_ACTV_COM2_COARSE_DOWN,
	CB_CMD_ACTV_COM2_COARSE_UP,
	CB_CMD_COM2_STANDBY_FLIP,

	CB_CMD_STDBY_NAV1_FINE_DOWN,
	CB_CMD_STDBY_NAV1_FINE_UP,
	CB_CMD_STDBY_NAV1_COARSE_DOWN,
	CB_CMD_STDBY_NAV1_COARSE_UP,
	CB_CMD_ACTV_NAV1_FINE_DOWN,
	CB_CMD_ACTV_NAV1_FINE_UP,
	CB_CMD_ACTV_NAV1_COARSE_DOWN,
	CB_CMD_ACTV_NAV1_COARSE_UP,
	CB_CMD_NAV1_STANDBY_FLIP,

	CB_CMD_STDBY_NAV2_FINE_DOWN,
	CB_CMD_STDBY_NAV2_FINE_UP,
	CB_CMD_STDBY_NAV2_COARSE_DOWN,
	CB_CMD_STDBY_NAV2_COARSE_UP,
	CB_CMD_ACTV_NAV2_FINE_DOWN,
	CB_CMD_ACTV_NAV2_FINE_UP,
	CB_CMD_ACTV_NAV2_COARSE_DOWN,
	CB_CMD_ACTV_NAV2_COARSE_UP,
	CB_CMD_NAV2_STANDBY_FLIP,
	CB_CMD_ADF1_STANDBY_FLIP,
	CB_CMD_ADF2_STANDBY_FLIP,

	CB_CMD_ACTV_ADF1_HUNDREDS_DOWN,
	CB_CMD_ACTV_ADF1_HUNDREDS_UP,
	CB_CMD_ACTV_ADF1_TENS_DOWN,
	CB_CMD_ACTV_ADF1_TENS_UP,
	CB_CMD_ACTV_ADF1_ONES_DOWN,
	CB_CMD_ACTV_ADF1_ONES_UP,
	CB_CMD_ACTV_ADF2_HUNDREDS_DOWN,
	CB_CMD_ACTV_ADF2_HUNDREDS_UP,
	CB_CMD_ACTV_ADF2_TENS_DOWN,
	CB_CMD_ACTV_ADF2_TENS_UP,
	CB_CMD_ACTV_ADF2_ONES_DOWN,
	CB_CMD_ACTV_ADF2_ONES_UP,
	CB_CMD_XPDR_THOUSANDS_DOWN,
	CB_CMD_XPDR_THOUSANDS_UP,
	CB_CMD_XPDR_HUNDREDS_DOWN,
	CB_CMD_XPDR_HUNDREDS_UP,
	CB_CMD_XPDR_TENS_DOWN,
	CB_CMD_XPDR_TENS_UP,
	CB_CMD_XPDR_ONES_DOWN,
	CB_CMD_XPDR_ONES_UP

};

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
static struct cb_thread_data *gPtrThreadData;
static unsigned char buf[CB_IN_BUF_SIZE];
static unsigned char writeBuf[CB_OUT_BUF_SIZE];
static unsigned char writeBufPrev[CB_OUT_BUF_SIZE];
static char tmp[100];
static uint32_t gUpperKnob = 0;
static uint32_t gLowerKnob = 0;


/* RADIO PANEL Command Refs */
XPLMCommandRef gCbStdbyCOM1FineDownCmdRef = NULL;
XPLMCommandRef gCbStdbyCOM1FineUpCmdRef = NULL;
XPLMCommandRef gCbStdbyCOM1CoarseDownCmdRef = NULL;
XPLMCommandRef gCbStdbyCOM1CoarseUpCmdRef = NULL;
XPLMCommandRef gCbActvCOM1FineDownCmdRef = NULL;
XPLMCommandRef gCbActvCOM1FineUpCmdRef = NULL;
XPLMCommandRef gCbActvCOM1CoarseDownCmdRef = NULL;
XPLMCommandRef gCbActvCOM1CoarseUpCmdRef = NULL;
XPLMCommandRef gCbCOM1StbyFlipCmdRef = NULL;

XPLMCommandRef gCbStdbyCOM2FineDownCmdRef = NULL;
XPLMCommandRef gCbStdbyCOM2FineUpCmdRef = NULL;
XPLMCommandRef gCbStdbyCOM2CoarseDownCmdRef = NULL;
XPLMCommandRef gCbStdbyCOM2CoarseUpCmdRef = NULL;
XPLMCommandRef gCbActvCOM2FineDownCmdRef = NULL;
XPLMCommandRef gCbActvCOM2FineUpCmdRef = NULL;
XPLMCommandRef gCbActvCOM2CoarseDownCmdRef = NULL;
XPLMCommandRef gCbActvCOM2CoarseUpCmdRef = NULL;
XPLMCommandRef gCbCOM2StbyFlipCmdRef = NULL;

XPLMCommandRef gCbStdbyNAV1FineDownCmdRef = NULL;
XPLMCommandRef gCbStdbyNAV1FineUpCmdRef = NULL;
XPLMCommandRef gCbStdbyNAV1CoarseDownCmdRef = NULL;
XPLMCommandRef gCbStdbyNAV1CoarseUpCmdRef = NULL;
XPLMCommandRef gCbActvNAV1FineDownCmdRef = NULL;
XPLMCommandRef gCbActvNAV1FineUpCmdRef = NULL;
XPLMCommandRef gCbActvNAV1CoarseDownCmdRef = NULL;
XPLMCommandRef gCbActvNAV1CoarseUpCmdRef = NULL;
XPLMCommandRef gCbNAV1StbyFlipCmdRef = NULL;

XPLMCommandRef gCbStdbyNAV2FineDownCmdRef = NULL;
XPLMCommandRef gCbStdbyNAV2FineUpCmdRef = NULL;
XPLMCommandRef gCbStdbyNAV2CoarseDownCmdRef = NULL;
XPLMCommandRef gCbStdbyNAV2CoarseUpCmdRef = NULL;
XPLMCommandRef gCbActvNAV2FineDownCmdRef = NULL;
XPLMCommandRef gCbActvNAV2FineUpCmdRef = NULL;
XPLMCommandRef gCbActvNAV2CoarseDownCmdRef = NULL;
XPLMCommandRef gCbActvNAV2CoarseUpCmdRef = NULL;
XPLMCommandRef gCbNAV2StbyFlipCmdRef = NULL;

XPLMCommandRef gCbActvADF1HundredsUpCmdRef = NULL;
XPLMCommandRef gCbActvADF1HundredsDownCmdRef = NULL;
XPLMCommandRef gCbActvADF1TensUpCmdRef = NULL;
XPLMCommandRef gCbActvADF1TensDownCmdRef = NULL;
XPLMCommandRef gCbActvADF1OnesUpCmdRef = NULL;
XPLMCommandRef gCbActvADF1OnesDownCmdRef = NULL;
XPLMCommandRef gCbADF1StbyFlipCmdRef = NULL;

XPLMCommandRef gCbActvADF2HundredsUpCmdRef = NULL;
XPLMCommandRef gCbActvADF2HundredsDownCmdRef = NULL;
XPLMCommandRef gCbActvADF2TensUpCmdRef = NULL;
XPLMCommandRef gCbActvADF2TensDownCmdRef = NULL;
XPLMCommandRef gCbActvADF2OnesUpCmdRef = NULL;
XPLMCommandRef gCbActvADF2OnesDownCmdRef = NULL;
XPLMCommandRef gCbADF2StbyFlipCmdRef = NULL;

XPLMCommandRef gCbXpdrThousandsUpCmdRef = NULL;
XPLMCommandRef gCbXpdrThousandsDownCmdRef = NULL;
XPLMCommandRef gCbXpdrHundredsUpCmdRef = NULL;
XPLMCommandRef gCbXpdrHundredsDownCmdRef = NULL;
XPLMCommandRef gCbXpdrTensUpCmdRef = NULL;
XPLMCommandRef gCbXpdrTensDownCmdRef = NULL;
XPLMCommandRef gCbXpdrOnesUpCmdRef = NULL;
XPLMCommandRef gCbXpdrOnesDownCmdRef = NULL;

/* RADIO PANEL Data Refs */
XPLMDataRef gCbCOM1FreqHzDataRef = NULL;
XPLMDataRef gCbCOM1StdbyFreqHzDataRef = NULL;

XPLMDataRef gCbCOM2FreqHzDataRef = NULL;
XPLMDataRef gCbCOM2StdbyFreqHzDataRef = NULL;

XPLMDataRef gCbNAV1FreqHzDataRef = NULL;
XPLMDataRef gCbNAV1StdbyFreqHzDataRef = NULL;

XPLMDataRef gCbNAV2FreqHzDataRef = NULL;
XPLMDataRef gCbNAV2StdbyFreqHzDataRef = NULL;

XPLMDataRef gCbXpdrCodeDataRef = NULL;
XPLMDataRef gCbQNHCodeDataRef = NULL;
XPLMDataRef gCbADF1FreqHzDataRef = NULL;
XPLMDataRef gCbADF1StdbyFreqHzDataRef = NULL;
XPLMDataRef gCbADF2FreqHzDataRef = NULL;
XPLMDataRef gCbADF2StdbyFreqHzDataRef = NULL;
XPLMDataRef gCbNAV1DmeDistanceNmDataRef = NULL;
XPLMDataRef gCbNAV1DmeSpeedKtsDataRef = NULL;
XPLMDataRef gCbNAV2DmeDistanceNmDataRef = NULL;
XPLMDataRef gCbNAV2DmeSpeedKtsDataRef = NULL;
XPLMDataRef gCbDmeSlaveSourceDataRef = NULL;

/* RADIO PANEL Global variables */
uint32_t gCbCOM1StbyFreq = 0;
uint32_t gCbCOM1Freq = 0;

uint32_t gCbCOM2StbyFreq = 0;
uint32_t gCbCOM2Freq = 0;

uint32_t gCbNAV1StbyFreq = 0;
uint32_t gCbNAV1Freq = 0;

uint32_t gCbNAV2StbyFreq = 0;
uint32_t gCbNAV2Freq = 0;

uint32_t gCbXpdrCode = 0;
float    gCbQNHCode = 0;
uint32_t gCbADF1FreqHz = 0;
uint32_t gCbADF1StdbyFreqHz = 0;
uint32_t gCbADF2FreqHz = 0;
uint32_t gCbADF2StdbyFreqHz = 0;
uint32_t gCbNAV1DmeDistanceNm = 0;
uint32_t gCbNAV1DmeSpeedKts = 0;
uint32_t gCbNAV2DmeDistanceNm = 0;
uint32_t gCbNAV2DmeSpeedKts = 0;
uint32_t gCbDmeSlaveSource = 0;

static int powerOn = 0;


int ColomboardPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void *            inRefcon) {
//	XPLMDebugString("-> CP: ColomboardPanelCommandHandler: start.\n");
//	char Buffer[256];
//	sprintf(Buffer,"Cmdh handler: 0x%08x, %d, 0x%08x\n", inCommand, inPhase, inRefcon);
//	XPLMDebugString(Buffer);
	int status = CMD_PASS_EVENT;

 switch ((int)(inRefcon)) {
		case CB_CMD_STDBY_COM1_FINE_DOWN:
		case CB_CMD_STDBY_COM1_FINE_UP:
		case CB_CMD_STDBY_COM1_COARSE_DOWN:
		case CB_CMD_STDBY_COM1_COARSE_UP:
			gCbCOM1StbyFreq = (XPLMGetDatai(gCbCOM1StdbyFreqHzDataRef));
			gCbCOM1Freq = (XPLMGetDatai(gCbCOM1FreqHzDataRef));
			break;
		case CB_CMD_ACTV_COM1_FINE_DOWN:
		case CB_CMD_ACTV_COM1_FINE_UP:
		case CB_CMD_ACTV_COM1_COARSE_DOWN:
		case CB_CMD_ACTV_COM1_COARSE_UP:
			gCbCOM1StbyFreq = (XPLMGetDatai(gCbCOM1StdbyFreqHzDataRef));
			gCbCOM1Freq = (XPLMGetDatai(gCbCOM1StdbyFreqHzDataRef));
			break;
		case CB_CMD_COM1_STANDBY_FLIP:
			gCbCOM1StbyFreq = (XPLMGetDatai(gCbCOM1FreqHzDataRef));
			gCbCOM1Freq = (XPLMGetDatai(gCbCOM1FreqHzDataRef));
			break;
		case CB_CMD_STDBY_COM2_FINE_DOWN:
		case CB_CMD_STDBY_COM2_FINE_UP:
		case CB_CMD_STDBY_COM2_COARSE_DOWN:
		case CB_CMD_STDBY_COM2_COARSE_UP:
			gCbCOM2StbyFreq = (XPLMGetDatai(gCbCOM2StdbyFreqHzDataRef));
			gCbCOM2Freq = (XPLMGetDatai(gCbCOM2FreqHzDataRef));
			break;
		case CB_CMD_ACTV_COM2_FINE_DOWN:
		case CB_CMD_ACTV_COM2_FINE_UP:
		case CB_CMD_ACTV_COM2_COARSE_DOWN:
		case CB_CMD_ACTV_COM2_COARSE_UP:
			gCbCOM2StbyFreq = (XPLMGetDatai(gCbCOM2StdbyFreqHzDataRef));
			gCbCOM2Freq = (XPLMGetDatai(gCbCOM2StdbyFreqHzDataRef));
			break;
		case CB_CMD_COM2_STANDBY_FLIP:
			gCbCOM2StbyFreq = (XPLMGetDatai(gCbCOM2FreqHzDataRef));
			gCbCOM2Freq = (XPLMGetDatai(gCbCOM2FreqHzDataRef));
			break;
		case CB_CMD_STDBY_NAV1_FINE_DOWN:
		case CB_CMD_STDBY_NAV1_FINE_UP:
		case CB_CMD_STDBY_NAV1_COARSE_DOWN:
		case CB_CMD_STDBY_NAV1_COARSE_UP:
			gCbNAV1StbyFreq = (XPLMGetDatai(gCbNAV1StdbyFreqHzDataRef));
			gCbNAV1Freq = (XPLMGetDatai(gCbNAV1FreqHzDataRef));
			break;
		case CB_CMD_ACTV_NAV1_FINE_DOWN:
		case CB_CMD_ACTV_NAV1_FINE_UP:
		case CB_CMD_ACTV_NAV1_COARSE_DOWN:
		case CB_CMD_ACTV_NAV1_COARSE_UP:
			gCbNAV1StbyFreq = (XPLMGetDatai(gCbNAV1StdbyFreqHzDataRef));
			gCbNAV1Freq = (XPLMGetDatai(gCbNAV1StdbyFreqHzDataRef));
			break;
		case CB_CMD_NAV1_STANDBY_FLIP:
			gCbNAV1StbyFreq = (XPLMGetDatai(gCbNAV1FreqHzDataRef));
			gCbNAV1Freq = (XPLMGetDatai(gCbNAV1FreqHzDataRef));
			break;
		case CB_CMD_STDBY_NAV2_FINE_DOWN:
		case CB_CMD_STDBY_NAV2_FINE_UP:
		case CB_CMD_STDBY_NAV2_COARSE_DOWN:
		case CB_CMD_STDBY_NAV2_COARSE_UP:
			gCbNAV2StbyFreq = (XPLMGetDatai(gCbNAV2StdbyFreqHzDataRef));
			gCbNAV2Freq = (XPLMGetDatai(gCbNAV2FreqHzDataRef));
			break;
		case CB_CMD_ACTV_NAV2_FINE_DOWN:
		case CB_CMD_ACTV_NAV2_FINE_UP:
		case CB_CMD_ACTV_NAV2_COARSE_DOWN:
		case CB_CMD_ACTV_NAV2_COARSE_UP:
			gCbNAV2StbyFreq = (XPLMGetDatai(gCbNAV2StdbyFreqHzDataRef));
			gCbNAV2Freq = (XPLMGetDatai(gCbNAV2StdbyFreqHzDataRef));
			break;
		case CB_CMD_NAV2_STANDBY_FLIP:
			gCbNAV2StbyFreq = (XPLMGetDatai(gCbNAV2FreqHzDataRef));
			gCbNAV2Freq = (XPLMGetDatai(gCbNAV2FreqHzDataRef));
			break;
		case CB_CMD_ACTV_ADF1_HUNDREDS_DOWN:
		case CB_CMD_ACTV_ADF1_HUNDREDS_UP:
		case CB_CMD_ACTV_ADF1_TENS_DOWN:
		case CB_CMD_ACTV_ADF1_TENS_UP:
		case CB_CMD_ACTV_ADF1_ONES_DOWN:
		case CB_CMD_ACTV_ADF1_ONES_UP:
			gCbADF1FreqHz = (XPLMGetDatai(gCbADF1FreqHzDataRef));
			gCbADF1StdbyFreqHz = (XPLMGetDatai(gCbADF1StdbyFreqHzDataRef));
			break;
		case CB_CMD_ACTV_ADF2_HUNDREDS_DOWN:
		case CB_CMD_ACTV_ADF2_HUNDREDS_UP:
		case CB_CMD_ACTV_ADF2_TENS_DOWN:
		case CB_CMD_ACTV_ADF2_TENS_UP:
		case CB_CMD_ACTV_ADF2_ONES_DOWN:
		case CB_CMD_ACTV_ADF2_ONES_UP:
			gCbADF2FreqHz = (XPLMGetDatai(gCbADF2FreqHzDataRef));
			gCbADF2StdbyFreqHz = (XPLMGetDatai(gCbADF2StdbyFreqHzDataRef));
			break;
		case CB_CMD_XPDR_THOUSANDS_DOWN:
		case CB_CMD_XPDR_THOUSANDS_UP:
		case CB_CMD_XPDR_HUNDREDS_DOWN:
		case CB_CMD_XPDR_HUNDREDS_UP:
		case CB_CMD_XPDR_TENS_DOWN:
		case CB_CMD_XPDR_TENS_UP:
		case CB_CMD_XPDR_ONES_DOWN:
		case CB_CMD_XPDR_ONES_UP:
			gCbXpdrCode = (XPLMGetDatai(gCbXpdrCodeDataRef));
			break;
		default:
			break;
 }

 return status;
}

void cb_process_power(uint32_t gPanelBatteryOn, uint32_t gPanelGeneratorOn, uint32_t gPanelAvionicsOn) {
	if (gPanelGeneratorOn || gPanelBatteryOn) {
		if (gPanelAvionicsOn == 1) {
			powerOn = 1;
		} else {
			powerOn = 0;
		}
	} else {
		powerOn = 0;
	}

}

int cb_has_changed(unsigned char a[], unsigned char b[]) {
	int i = 0;
	for (i = 0; i < CB_OUT_BUF_SIZE; i++) {
		if (a[i] != b[i]) {
			return 1;
		}
	}
	return 0;
}

inline void cb_upper_led_update(uint32_t x, uint32_t y, uint32_t upperDecimal, uint32_t upperPos1, uint32_t upperPos2,
		uint32_t z, uint32_t r, uint32_t lowerDecimal, uint32_t lowerPos1, uint32_t lowerPos2, uint8_t m[]) {
    m[0] = 0x00;
    m[1] = ((x >> 16) & 0x0F) | upperPos1;
    m[2] = ((x >> 12) & 0x0F) | upperPos2;
    m[3] = ((x >>  8) & 0x0F) | upperDecimal;
    m[4] = ((x >>  4) & 0x0F);
    m[5] = ((x >>  0) & 0x0F);
    m[6] = ((y >> 16) & 0x0F) | upperPos1;
    m[7] = ((y >> 12) & 0x0F) | upperPos2;
    m[8] = ((y >>  8) & 0x0F) | upperDecimal;
    m[9] = ((y >>  4) & 0x0F);
    m[10] = ((y >>  0) & 0x0F);
    m[11] = ((z >> 16) & 0x0F) | lowerPos1;
    m[12] = ((z >> 12) & 0x0F) | lowerPos2;
    m[13] = ((z >>  8) & 0x0F) | lowerDecimal;
    m[14] = ((z >>  4) & 0x0F);
    m[15] = ((z >>  0) & 0x0F);
    m[16] = ((r >> 16) & 0x0F) | lowerPos1;
    m[17] = ((r >> 12) & 0x0F) | lowerPos2;
    m[18] = ((r >>  8) & 0x0F) | lowerDecimal;
    m[19] = ((r >>  4) & 0x0F);
    m[20] = ((r >>  0) & 0x0F);
    m[21] = 0x00;
    m[22] = 0x00;
}

void cb_process_coarse_right(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gCbStdbyCOM1CoarseUpCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gCbStdbyCOM2CoarseUpCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gCbStdbyNAV1CoarseUpCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gCbStdbyNAV2CoarseUpCmdRef);
		break;
	case 0x000010:
		XPLMCommandOnce(gCbActvADF1HundredsUpCmdRef);
		break;
	case 0x000800:
		XPLMCommandOnce(gCbActvADF2HundredsUpCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		//XPLMCommandOnce(gCbStdbyDMECoarseUpCmdRef);
		break;
	case 0x000040:
	case 0x002000:
		if (((gCbXpdrCode / 100) % 10) == 7) {
			XPLMCommandOnce(gCbXpdrThousandsUpCmdRef);
		}
		XPLMCommandOnce(gCbXpdrHundredsUpCmdRef);
		break;
	default:
		break;
	}
}

void cb_process_coarse_left(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gCbStdbyCOM1CoarseDownCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gCbStdbyCOM2CoarseDownCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gCbStdbyNAV1CoarseDownCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gCbStdbyNAV2CoarseDownCmdRef);
		break;
	case 0x000010:
		XPLMCommandOnce(gCbActvADF1HundredsDownCmdRef);
		break;
	case 0x000800:
		XPLMCommandOnce(gCbActvADF2HundredsDownCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		if (((gCbXpdrCode / 100) % 10) == 0) {
			XPLMCommandOnce(gCbXpdrThousandsDownCmdRef);
		}
		XPLMCommandOnce(gCbXpdrHundredsDownCmdRef);
		break;
	default:
		break;
	}
}

void cb_process_fine_right(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gCbStdbyCOM1FineUpCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gCbStdbyCOM2FineUpCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gCbStdbyNAV1FineUpCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gCbStdbyNAV2FineUpCmdRef);
		break;
	case 0x000010:
		if ((gCbADF1StdbyFreqHz % 10) == 9) {
			XPLMCommandOnce(gCbActvADF1TensUpCmdRef);
		}
		XPLMCommandOnce(gCbActvADF1OnesUpCmdRef);
		break;
	case 0x000800:
		if ((gCbADF2StdbyFreqHz % 10) == 9) {
			XPLMCommandOnce(gCbActvADF2TensUpCmdRef);
		}
		XPLMCommandOnce(gCbActvADF2OnesUpCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		if ((gCbXpdrCode % 10) == 7) {
			XPLMCommandOnce(gCbXpdrTensUpCmdRef);
		}
		XPLMCommandOnce(gCbXpdrOnesUpCmdRef);
		break;
	default:
		break;
	}
}

void cb_process_fine_left(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gCbStdbyCOM1FineDownCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gCbStdbyCOM2FineDownCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gCbStdbyNAV1FineDownCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gCbStdbyNAV2FineDownCmdRef);
		break;
	case 0x000010:
		if ((gCbADF1StdbyFreqHz % 10) == 0) {
			XPLMCommandOnce(gCbActvADF1TensDownCmdRef);
		}
		XPLMCommandOnce(gCbActvADF1OnesDownCmdRef);
		break;
	case 0x000800:
		if ((gCbADF2StdbyFreqHz % 10) == 0) {
			XPLMCommandOnce(gCbActvADF2TensDownCmdRef);
		}
		XPLMCommandOnce(gCbActvADF2OnesDownCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		if ((gCbXpdrCode % 10) == 0) {
			XPLMCommandOnce(gCbXpdrTensDownCmdRef);
		}
		XPLMCommandOnce(gCbXpdrOnesDownCmdRef);
		break;
	default:
		break;
	}
}

void cb_process_switch(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gCbCOM1StbyFlipCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gCbCOM2StbyFlipCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gCbNAV1StbyFlipCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gCbNAV2StbyFlipCmdRef);
		break;
	case 0x000010:
		XPLMCommandOnce(gCbADF1StbyFlipCmdRef);
		break;
	case 0x000800:
		XPLMCommandOnce(gCbADF2StbyFlipCmdRef);
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

int cb_process(uint32_t msg) {
    //sprintf(tmp, "-> CP: cb_controller.cb_process: msg: %d\n", msg);
	//XPLMDebugString(tmp);
	int res = 0;
    gUpperKnob = msg & CB_READ_UPPER_KNOB_MODE_MASK;
    gLowerKnob = msg & CB_READ_LOWER_KNOB_MODE_MASK;
    uint32_t upperFineTuning = msg & CB_READ_UPPER_FINE_TUNING_MASK;
    uint32_t upperCoarseTuning = msg & CB_READ_UPPER_COARSE_TUNING_MASK;
    uint32_t lowerFineTuning = msg & CB_READ_LOWER_FINE_TUNING_MASK;
    uint32_t lowerCoarseTuning = msg & CB_READ_LOWER_COARSE_TUNING_MASK;
    uint32_t upperStby = msg & CB_READ_UPPER_ACT_STBY;
    uint32_t lowerStby = msg & CB_READ_LOWER_ACT_STBY;

    if (upperCoarseTuning || upperFineTuning) {
		if (upperCoarseTuning == CB_READ_UPPER_COARSE_RIGHT) {
			cb_process_coarse_right(gUpperKnob);
		} else if (upperCoarseTuning == CB_READ_UPPER_COARSE_LEFT) {
			cb_process_coarse_left(gUpperKnob);
		} else if (upperFineTuning == CB_READ_UPPER_FINE_RIGHT) {
			cb_process_fine_right(gUpperKnob);
		} else if (upperFineTuning == CB_READ_UPPER_FINE_LEFT) {
			cb_process_fine_left(gUpperKnob);
		}
    }
    if (upperStby) {
    	cb_process_switch(gUpperKnob);
    }
    if (lowerCoarseTuning || lowerFineTuning) {
		if (lowerCoarseTuning == CB_READ_LOWER_COARSE_RIGHT) {
			cb_process_coarse_right(gLowerKnob);
		} else if (lowerCoarseTuning == CB_READ_LOWER_COARSE_LEFT) {
			cb_process_coarse_left(gLowerKnob);
		} else if (lowerFineTuning == CB_READ_LOWER_FINE_RIGHT) {
			cb_process_fine_right(gLowerKnob);
		} else if (lowerFineTuning == CB_READ_LOWER_FINE_LEFT) {
			cb_process_fine_left(gLowerKnob);
		}
    }
    if (lowerStby) {
    	cb_process_switch(gLowerKnob);
    }
    return res;
}

void cb_update_datarefs() {
    gCbCOM1StbyFreq = (XPLMGetDatai(gCbCOM1StdbyFreqHzDataRef));
    gCbCOM1Freq = (XPLMGetDatai(gCbCOM1FreqHzDataRef));

    gCbCOM2StbyFreq = (XPLMGetDatai(gCbCOM2StdbyFreqHzDataRef));
    gCbCOM2Freq = (XPLMGetDatai(gCbCOM2FreqHzDataRef));

    gCbNAV1StbyFreq = (XPLMGetDatai(gCbNAV1StdbyFreqHzDataRef));
    gCbNAV1Freq = (XPLMGetDatai(gCbNAV1FreqHzDataRef));

    gCbNAV2StbyFreq = (XPLMGetDatai(gCbNAV2StdbyFreqHzDataRef));
    gCbNAV2Freq = (XPLMGetDatai(gCbNAV2FreqHzDataRef));

    gCbXpdrCode = (XPLMGetDatai(gCbXpdrCodeDataRef));
    gCbQNHCode = (XPLMGetDataf(gCbQNHCodeDataRef));
    gCbADF1FreqHz = (XPLMGetDatai(gCbADF1FreqHzDataRef));
    gCbADF1StdbyFreqHz = (XPLMGetDatai(gCbADF1StdbyFreqHzDataRef));
    gCbADF2FreqHz = (XPLMGetDatai(gCbADF2FreqHzDataRef));
    gCbADF2StdbyFreqHz = (XPLMGetDatai(gCbADF2StdbyFreqHzDataRef));
    gCbNAV1DmeDistanceNm = (XPLMGetDatai(gCbNAV1DmeDistanceNmDataRef));
    gCbNAV1DmeSpeedKts = (XPLMGetDatai(gCbNAV1DmeSpeedKtsDataRef));
    gCbNAV2DmeDistanceNm = (XPLMGetDatai(gCbNAV2DmeDistanceNmDataRef));
    gCbNAV2DmeSpeedKts = (XPLMGetDatai(gCbNAV2DmeSpeedKtsDataRef));
    gCbDmeSlaveSource = (XPLMGetDatai(gCbDmeSlaveSourceDataRef));

}

void cb_init() {
	XPLMDebugString("-> CP: cb_controller.cb_init.\n");
	gCbStdbyCOM1FineDownCmdRef         = XPLMFindCommand(sCB_STDBY_COM1_FINE_DOWN_CR);
	gCbStdbyCOM1FineUpCmdRef           = XPLMFindCommand(sCB_STDBY_COM1_FINE_UP_CR);
	gCbStdbyCOM1CoarseDownCmdRef       = XPLMFindCommand(sCB_STDBY_COM1_COARSE_DOWN_CR);
	gCbStdbyCOM1CoarseUpCmdRef         = XPLMFindCommand(sCB_STDBY_COM1_COARSE_UP_CR);
	gCbActvCOM1FineDownCmdRef         = XPLMFindCommand(sCB_ACTV_COM1_FINE_DOWN_CR);
	gCbActvCOM1FineUpCmdRef           = XPLMFindCommand(sCB_ACTV_COM1_FINE_UP_CR);
	gCbActvCOM1CoarseDownCmdRef       = XPLMFindCommand(sCB_ACTV_COM1_COARSE_DOWN_CR);
	gCbActvCOM1CoarseUpCmdRef         = XPLMFindCommand(sCB_ACTV_COM1_COARSE_UP_CR);
	gCbCOM1StbyFlipCmdRef              = XPLMFindCommand(sCB_COM1_STBY_FLIP_CR);

	gCbStdbyCOM2FineDownCmdRef         = XPLMFindCommand(sCB_STDBY_COM2_FINE_DOWN_CR);
	gCbStdbyCOM2FineUpCmdRef           = XPLMFindCommand(sCB_STDBY_COM2_FINE_UP_CR);
	gCbStdbyCOM2CoarseDownCmdRef       = XPLMFindCommand(sCB_STDBY_COM2_COARSE_DOWN_CR);
	gCbStdbyCOM2CoarseUpCmdRef         = XPLMFindCommand(sCB_STDBY_COM2_COARSE_UP_CR);
	gCbActvCOM2FineDownCmdRef         = XPLMFindCommand(sCB_ACTV_COM2_FINE_DOWN_CR);
	gCbActvCOM2FineUpCmdRef           = XPLMFindCommand(sCB_ACTV_COM2_FINE_UP_CR);
	gCbActvCOM2CoarseDownCmdRef       = XPLMFindCommand(sCB_ACTV_COM2_COARSE_DOWN_CR);
	gCbActvCOM2CoarseUpCmdRef         = XPLMFindCommand(sCB_ACTV_COM2_COARSE_UP_CR);
	gCbCOM2StbyFlipCmdRef              = XPLMFindCommand(sCB_COM2_STBY_FLIP_CR);

    XPLMRegisterCommandHandler(gCbStdbyCOM1FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM1_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyCOM1FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM1_FINE_UP);
    XPLMRegisterCommandHandler(gCbStdbyCOM1CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyCOM1CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM1_COARSE_UP);
    XPLMRegisterCommandHandler(gCbActvCOM1FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM1_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbActvCOM1FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM1_FINE_UP);
    XPLMRegisterCommandHandler(gCbActvCOM1CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbActvCOM1CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM1_COARSE_UP);
    XPLMRegisterCommandHandler(gCbCOM1StbyFlipCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_COM1_STANDBY_FLIP);

    gCbCOM1FreqHzDataRef        = XPLMFindDataRef(sCB_COM1_FREQ_HZ_DR);
    gCbCOM1StdbyFreqHzDataRef   = XPLMFindDataRef(sCB_COM1_STDBY_FREQ_HZ_DR);

    gCbCOM1StbyFreq = (XPLMGetDatai(gCbCOM1StdbyFreqHzDataRef));
    gCbCOM1Freq = (XPLMGetDatai(gCbCOM1FreqHzDataRef));

    XPLMRegisterCommandHandler(gCbStdbyCOM2FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM2_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyCOM2FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM2_FINE_UP);
    XPLMRegisterCommandHandler(gCbStdbyCOM2CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyCOM2CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_COM2_COARSE_UP);
    XPLMRegisterCommandHandler(gCbActvCOM2FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM2_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbActvCOM2FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM2_FINE_UP);
    XPLMRegisterCommandHandler(gCbActvCOM2CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbActvCOM2CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_COM2_COARSE_UP);
    XPLMRegisterCommandHandler(gCbCOM2StbyFlipCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_COM2_STANDBY_FLIP);

    gCbCOM2FreqHzDataRef        = XPLMFindDataRef(sCB_COM2_FREQ_HZ_DR);
    gCbCOM2StdbyFreqHzDataRef   = XPLMFindDataRef(sCB_COM2_STDBY_FREQ_HZ_DR);

    gCbCOM2StbyFreq = (XPLMGetDatai(gCbCOM2StdbyFreqHzDataRef));
    gCbCOM2Freq = (XPLMGetDatai(gCbCOM2FreqHzDataRef));

	gCbStdbyNAV1FineDownCmdRef         = XPLMFindCommand(sCB_STDBY_NAV1_FINE_DOWN_CR);
	gCbStdbyNAV1FineUpCmdRef           = XPLMFindCommand(sCB_STDBY_NAV1_FINE_UP_CR);
	gCbStdbyNAV1CoarseDownCmdRef       = XPLMFindCommand(sCB_STDBY_NAV1_COARSE_DOWN_CR);
	gCbStdbyNAV1CoarseUpCmdRef         = XPLMFindCommand(sCB_STDBY_NAV1_COARSE_UP_CR);
	gCbActvNAV1FineDownCmdRef         = XPLMFindCommand(sCB_ACTV_NAV1_FINE_DOWN_CR);
	gCbActvNAV1FineUpCmdRef           = XPLMFindCommand(sCB_ACTV_NAV1_FINE_UP_CR);
	gCbActvNAV1CoarseDownCmdRef       = XPLMFindCommand(sCB_ACTV_NAV1_COARSE_DOWN_CR);
	gCbActvNAV1CoarseUpCmdRef         = XPLMFindCommand(sCB_ACTV_NAV1_COARSE_UP_CR);
	gCbNAV1StbyFlipCmdRef              = XPLMFindCommand(sCB_NAV1_STBY_FLIP_CR);

	gCbStdbyNAV2FineDownCmdRef         = XPLMFindCommand(sCB_STDBY_NAV2_FINE_DOWN_CR);
	gCbStdbyNAV2FineUpCmdRef           = XPLMFindCommand(sCB_STDBY_NAV2_FINE_UP_CR);
	gCbStdbyNAV2CoarseDownCmdRef       = XPLMFindCommand(sCB_STDBY_NAV2_COARSE_DOWN_CR);
	gCbStdbyNAV2CoarseUpCmdRef         = XPLMFindCommand(sCB_STDBY_NAV2_COARSE_UP_CR);
	gCbActvNAV2FineDownCmdRef         = XPLMFindCommand(sCB_ACTV_NAV2_FINE_DOWN_CR);
	gCbActvNAV2FineUpCmdRef           = XPLMFindCommand(sCB_ACTV_NAV2_FINE_UP_CR);
	gCbActvNAV2CoarseDownCmdRef       = XPLMFindCommand(sCB_ACTV_NAV2_COARSE_DOWN_CR);
	gCbActvNAV2CoarseUpCmdRef         = XPLMFindCommand(sCB_ACTV_NAV2_COARSE_UP_CR);
	gCbNAV2StbyFlipCmdRef              = XPLMFindCommand(sCB_NAV2_STBY_FLIP_CR);
	gCbADF1StbyFlipCmdRef              = XPLMFindCommand(sCB_ADF1_STBY_FLIP_CR);
	gCbADF2StbyFlipCmdRef              = XPLMFindCommand(sCB_ADF2_STBY_FLIP_CR);

    XPLMRegisterCommandHandler(gCbStdbyNAV1FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV1_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyNAV1FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV1_FINE_UP);
    XPLMRegisterCommandHandler(gCbStdbyNAV1CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyNAV1CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV1_COARSE_UP);
    XPLMRegisterCommandHandler(gCbActvNAV1FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV1_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbActvNAV1FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV1_FINE_UP);
    XPLMRegisterCommandHandler(gCbActvNAV1CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbActvNAV1CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV1_COARSE_UP);
    XPLMRegisterCommandHandler(gCbNAV1StbyFlipCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_NAV1_STANDBY_FLIP);

    gCbNAV1FreqHzDataRef        = XPLMFindDataRef(sCB_NAV1_FREQ_HZ_DR);
    gCbNAV1StdbyFreqHzDataRef   = XPLMFindDataRef(sCB_NAV1_STDBY_FREQ_HZ_DR);

    gCbNAV1StbyFreq = (XPLMGetDatai(gCbNAV1StdbyFreqHzDataRef));
    gCbNAV1Freq = (XPLMGetDatai(gCbNAV1FreqHzDataRef));

    XPLMRegisterCommandHandler(gCbStdbyNAV2FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV2_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyNAV2FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV2_FINE_UP);
    XPLMRegisterCommandHandler(gCbStdbyNAV2CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbStdbyNAV2CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_STDBY_NAV2_COARSE_UP);
    XPLMRegisterCommandHandler(gCbActvNAV2FineDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV2_FINE_DOWN);
    XPLMRegisterCommandHandler(gCbActvNAV2FineUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV2_FINE_UP);
    XPLMRegisterCommandHandler(gCbActvNAV2CoarseDownCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gCbActvNAV2CoarseUpCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ACTV_NAV2_COARSE_UP);
    XPLMRegisterCommandHandler(gCbNAV2StbyFlipCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_NAV2_STANDBY_FLIP);
    XPLMRegisterCommandHandler(gCbADF1StbyFlipCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ADF1_STANDBY_FLIP);
    XPLMRegisterCommandHandler(gCbADF2StbyFlipCmdRef, ColomboardPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) CB_CMD_ADF2_STANDBY_FLIP);

    gCbNAV2FreqHzDataRef        = XPLMFindDataRef(sCB_NAV2_FREQ_HZ_DR);
    gCbNAV2StdbyFreqHzDataRef   = XPLMFindDataRef(sCB_NAV2_STDBY_FREQ_HZ_DR);

    gCbNAV2StbyFreq = (XPLMGetDatai(gCbNAV2StdbyFreqHzDataRef));
    gCbNAV2Freq = (XPLMGetDatai(gCbNAV2FreqHzDataRef));

    gCbActvADF1HundredsUpCmdRef = XPLMFindCommand(sCB_ACTV_ADF1_HUNDREDS_UP_CR);
    gCbActvADF1HundredsDownCmdRef = XPLMFindCommand(sCB_ACTV_ADF1_HUNDREDS_DOWN_CR);
    gCbActvADF1TensUpCmdRef = XPLMFindCommand(sCB_ACTV_ADF1_TENS_UP_CR);
    gCbActvADF1TensDownCmdRef = XPLMFindCommand(sCB_ACTV_ADF1_TENS_DOWN_CR);
    gCbActvADF1OnesUpCmdRef = XPLMFindCommand(sCB_ACTV_ADF1_ONES_UP_CR);
    gCbActvADF1OnesDownCmdRef = XPLMFindCommand(sCB_ACTV_ADF1_ONES_DOWN_CR);

    gCbActvADF2HundredsUpCmdRef = XPLMFindCommand(sCB_ACTV_ADF2_HUNDREDS_UP_CR);
    gCbActvADF2HundredsDownCmdRef = XPLMFindCommand(sCB_ACTV_ADF2_HUNDREDS_DOWN_CR);
    gCbActvADF2TensUpCmdRef = XPLMFindCommand(sCB_ACTV_ADF2_TENS_UP_CR);
    gCbActvADF2TensDownCmdRef = XPLMFindCommand(sCB_ACTV_ADF2_TENS_DOWN_CR);
    gCbActvADF2OnesUpCmdRef = XPLMFindCommand(sCB_ACTV_ADF2_ONES_UP_CR);
    gCbActvADF2OnesDownCmdRef = XPLMFindCommand(sCB_ACTV_ADF2_ONES_DOWN_CR);

    gCbXpdrThousandsUpCmdRef = XPLMFindCommand(sCB_XPDR_THOUSANDS_UP_CR);
    gCbXpdrThousandsDownCmdRef = XPLMFindCommand(sCB_XPDR_THOUSANDS_DOWN_CR);
    gCbXpdrHundredsUpCmdRef = XPLMFindCommand(sCB_XPDR_HUNDREDS_UP_CR);
    gCbXpdrHundredsDownCmdRef = XPLMFindCommand(sCB_XPDR_HUNDREDS_DOWN_CR);
    gCbXpdrTensUpCmdRef = XPLMFindCommand(sCB_XPDR_TENS_UP_CR);
    gCbXpdrTensDownCmdRef = XPLMFindCommand(sCB_XPDR_TENS_DOWN_CR);
    gCbXpdrOnesUpCmdRef = XPLMFindCommand(sCB_XPDR_ONES_UP_CR);
    gCbXpdrOnesDownCmdRef = XPLMFindCommand(sCB_XPDR_ONES_DOWN_CR);

    gCbXpdrCodeDataRef = XPLMFindDataRef(sCB_XPDR_CODE_DR);
    gCbQNHCodeDataRef = XPLMFindDataRef(sCB_QNH_CODE_DR);
    gCbADF1FreqHzDataRef = XPLMFindDataRef(sCB_ADF1_FREQ_HZ_DR);
    gCbADF1StdbyFreqHzDataRef = XPLMFindDataRef(sCB_ADF1_STDBY_FREQ_HZ_DR);
    gCbADF2FreqHzDataRef = XPLMFindDataRef(sCB_ADF2_FREQ_HZ_DR);
    gCbADF2StdbyFreqHzDataRef = XPLMFindDataRef(sCB_ADF2_STDBY_FREQ_HZ_DR);
    gCbNAV1DmeDistanceNmDataRef = XPLMFindDataRef(sCB_NAV1_DME_DISTANCE_NM_DR);
    gCbNAV1DmeSpeedKtsDataRef = XPLMFindDataRef(sCB_NAV1_DME_SPEED_KTS_DR);
    gCbNAV2DmeDistanceNmDataRef = XPLMFindDataRef(sCB_NAV2_DME_DISTANCE_NM_DR);
    gCbNAV2DmeSpeedKtsDataRef = XPLMFindDataRef(sCB_NAV2_DME_SPEED_KTS_DR);
    gCbDmeSlaveSourceDataRef = XPLMFindDataRef(sCB_DME_SLAVE_SOURCE_DR);

    gCbXpdrCode = (XPLMGetDatai(gCbXpdrCodeDataRef));
    gCbQNHCode = (XPLMGetDataf(gCbQNHCodeDataRef));
    gCbADF1FreqHz = (XPLMGetDatai(gCbADF1FreqHzDataRef));
    gCbADF1StdbyFreqHz = (XPLMGetDatai(gCbADF1StdbyFreqHzDataRef));
    gCbADF2FreqHz = (XPLMGetDatai(gCbADF2FreqHzDataRef));
    gCbADF2StdbyFreqHz = (XPLMGetDatai(gCbADF2StdbyFreqHzDataRef));
    gCbNAV1DmeDistanceNm = (XPLMGetDatai(gCbNAV1DmeDistanceNmDataRef));
    gCbNAV1DmeSpeedKts = (XPLMGetDatai(gCbNAV1DmeSpeedKtsDataRef));
    gCbNAV2DmeDistanceNm = (XPLMGetDatai(gCbNAV2DmeDistanceNmDataRef));
    gCbNAV2DmeSpeedKts = (XPLMGetDatai(gCbNAV2DmeSpeedKtsDataRef));
    gCbDmeSlaveSource = (XPLMGetDatai(gCbDmeSlaveSourceDataRef));
}

void cb_prepare_write_buffer(int i, int j) {
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	uint32_t tmp3 = 0;
	uint32_t tmp4 = 0;
	uint32_t upperDecimal = 0;
	uint32_t upperPos1 = 0;
	uint32_t upperPos2 = 0;
	uint32_t lowerDecimal = 0;
	uint32_t lowerPos1 = 0;
	uint32_t lowerPos2 = 0;
	switch (gUpperKnob) {
	case 0x000001:
	case 0x000080:
		tmp1 = dec2bcd(gCbCOM1Freq, 5);
		tmp2 = dec2bcd(gCbCOM1StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000002:
	case 0x000100:
		tmp1 = dec2bcd(gCbCOM2Freq, 5);
		tmp2 = dec2bcd(gCbCOM2StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000004:
	case 0x000200:
		tmp1 = dec2bcd(gCbNAV1Freq, 5);
		tmp2 = dec2bcd(gCbNAV1StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000008:
	case 0x000400:
		tmp1 = dec2bcd(gCbNAV2Freq, 5);
		tmp2 = dec2bcd(gCbNAV2StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000010:
	case 0x000800:
		tmp1 = dec2bcd(gCbADF1FreqHz, 5);
		tmp2 = dec2bcd(gCbADF1StdbyFreqHz, 5);
		upperPos1 = 0xFF;
		upperPos2 = 0xFF;
		break;
	case 0x000020:
	case 0x001000:
		tmp1 = dec2bcd(gCbNAV1DmeDistanceNm, 5);
		tmp2 = dec2bcd(gCbNAV1DmeSpeedKts, 5);
		break;
	case 0x000040:
	case 0x002000:
		tmp1 = dec2bcd(gCbXpdrCode, 5);
		tmp2 = dec2bcd(round(gCbQNHCode * 100.0), 5);
		upperPos1 = 0xFF;
		break;
	default:
		break;
	}
	switch (gLowerKnob) {
	case 0x000001:
	case 0x000080:
		tmp3 = dec2bcd(gCbCOM1Freq, 5);
		tmp4 = dec2bcd(gCbCOM1StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000002:
	case 0x000100:
		tmp3 = dec2bcd(gCbCOM2Freq, 5);
		tmp4 = dec2bcd(gCbCOM2StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000004:
	case 0x000200:
		tmp3 = dec2bcd(gCbNAV1Freq, 5);
		tmp4 = dec2bcd(gCbNAV1StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000008:
	case 0x000400:
		tmp3 = dec2bcd(gCbNAV2Freq, 5);
		tmp4 = dec2bcd(gCbNAV2StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000010:
	case 0x000800:
		tmp3 = dec2bcd(gCbADF2FreqHz, 5);
		tmp4 = dec2bcd(gCbADF2StdbyFreqHz, 5);
		lowerPos1 = 0xFF;
		lowerPos2 = 0xFF;
		break;
	case 0x000020:
	case 0x001000:
		tmp3 = dec2bcd(gCbNAV2DmeDistanceNm, 5);
		tmp4 = dec2bcd(gCbNAV2DmeSpeedKts, 5);
//		tmp3 = dec2bcd(i % 100000, 5);
//		tmp4 = dec2bcd(j % 1000000, 5);
		break;
	case 0x000040:
	case 0x002000:
		tmp3 = dec2bcd(gCbXpdrCode, 5);
		tmp4 = dec2bcd(round(gCbQNHCode * 100.0), 5);
		lowerPos1 = 0xFF;
		break;
	default:
		break;
	}
	cb_upper_led_update(tmp1, tmp2, upperDecimal, upperPos1, upperPos2, tmp3, tmp4, lowerDecimal, lowerPos1, lowerPos2, writeBuf);
}


void *cbRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	int inReportBytesCount = 0;
	int i;

#if IBM
		Sleep(SLEEP_TIME * 5);
#else
		usleep(SLEEP_TIME * 5);
#endif

	gPtrThreadData = (struct cb_thread_data *) ptr_thread_data;

	int result = cb_panel_open();
	if (result < 0) {
		XPLMDebugString("-> CP: cb_controller.cbRun: shutdown thread.\n");
		gPtrThreadData->stop = 1;
		pthread_exit(NULL);
		return 0;
	}
	XPLMDebugString("-> CP: cb_controller.cbRun: panel opened.\n");

	// panel is open. now initialize datarefs.
	cb_init();
	cb_panel_write(cb_blank_panel);
	inReportBytesCount = cb_panel_read_non_blocking(buf);
	// URB_FUNCTION_CLASS_INTERFACE request should be 0x01 instead of 0x09
	cb_panel_write_empty();
	inReportBytesCount = cb_panel_read_non_blocking(buf);
	cb_panel_write(cb_zero_panel);
	inReportBytesCount = cb_panel_read_non_blocking(buf);

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
			cb_prepare_write_buffer(counter, counter2);
			inReportBytesCount = cb_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: cb_controller.cbRun: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				} else if (inReportBytesCount == 2) {
					counter2++;
					uint32_t msg = 0;
					msg += buf[1] << 16;
					msg += buf[0] << 8;
					cb_process(msg);
				} else {
					counter2++;
					uint32_t msg = 0;
					msg += buf[2] << 16;
					msg += buf[1] << 8;
					msg += buf[0];
					cb_process(msg);
				}
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// Update Panel. CRITICAL 20 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(50000, COUNTER2, loop_start_time)) {
			// Update local DataRefs.
			cb_update_datarefs();
			// update Panel.
			inReportBytesCount = cb_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: cb_controller.cbRun: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				} else if (inReportBytesCount == 2) {
					counter2++;
					uint32_t msg = 0;
					msg += buf[1] << 16;
					msg += buf[0] << 8;
					cb_process(msg);
				} else {
					uint32_t msg = 0;
					msg += buf[2] << 16;
					msg += buf[1] << 8;
					msg += buf[0];
					cb_process(msg);
				}
			}
			if (powerOn == 0) {
				for (i = 0; i < CB_OUT_BUF_SIZE; i++) {
					writeBuf[i] = cb_blank_panel[i];
				}
			}
			if (cb_has_changed(writeBuf, writeBufPrev)) {
				int i = 0;
				for(i = 0; i < CB_OUT_BUF_SIZE; i++) {
					writeBufPrev[i] = writeBuf[i];
				}
				cb_panel_write(writeBuf);
			}
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= MAX_DELAY_TIME) {
			XPLMDebugString("-> CP: cb_controller.cbRun: CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		} else {
			//writeConsole(0, 0, "CPU LOAD OK.");
		}

		// wait 1 milliseconds
#if IBM
		Sleep(SLEEP_TIME * 5);
#else
		usleep(SLEEP_TIME * 5);
#endif
	}
	cb_panel_close();
#if IBM
		Sleep(SLEEP_TIME * 5);
#else
		usleep(SLEEP_TIME * 5);
#endif
	pthread_exit(NULL);
	return 0;
}

