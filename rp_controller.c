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
	RP_CMD_STDBY_COM1_FINE_DOWN,
	RP_CMD_STDBY_COM1_FINE_UP,
	RP_CMD_STDBY_COM1_COARSE_DOWN,
	RP_CMD_STDBY_COM1_COARSE_UP,
	RP_CMD_ACTV_COM1_FINE_DOWN,
	RP_CMD_ACTV_COM1_FINE_UP,
	RP_CMD_ACTV_COM1_COARSE_DOWN,
	RP_CMD_ACTV_COM1_COARSE_UP,
	RP_CMD_COM1_STANDBY_FLIP,

	RP_CMD_STDBY_COM2_FINE_DOWN,
	RP_CMD_STDBY_COM2_FINE_UP,
	RP_CMD_STDBY_COM2_COARSE_DOWN,
	RP_CMD_STDBY_COM2_COARSE_UP,
	RP_CMD_ACTV_COM2_FINE_DOWN,
	RP_CMD_ACTV_COM2_FINE_UP,
	RP_CMD_ACTV_COM2_COARSE_DOWN,
	RP_CMD_ACTV_COM2_COARSE_UP,
	RP_CMD_COM2_STANDBY_FLIP,

	RP_CMD_STDBY_NAV1_FINE_DOWN,
	RP_CMD_STDBY_NAV1_FINE_UP,
	RP_CMD_STDBY_NAV1_COARSE_DOWN,
	RP_CMD_STDBY_NAV1_COARSE_UP,
	RP_CMD_ACTV_NAV1_FINE_DOWN,
	RP_CMD_ACTV_NAV1_FINE_UP,
	RP_CMD_ACTV_NAV1_COARSE_DOWN,
	RP_CMD_ACTV_NAV1_COARSE_UP,
	RP_CMD_NAV1_STANDBY_FLIP,

	RP_CMD_STDBY_NAV2_FINE_DOWN,
	RP_CMD_STDBY_NAV2_FINE_UP,
	RP_CMD_STDBY_NAV2_COARSE_DOWN,
	RP_CMD_STDBY_NAV2_COARSE_UP,
	RP_CMD_ACTV_NAV2_FINE_DOWN,
	RP_CMD_ACTV_NAV2_FINE_UP,
	RP_CMD_ACTV_NAV2_COARSE_DOWN,
	RP_CMD_ACTV_NAV2_COARSE_UP,
	RP_CMD_NAV2_STANDBY_FLIP,
	RP_CMD_ADF1_STANDBY_FLIP,
	RP_CMD_ADF2_STANDBY_FLIP,

	RP_CMD_ACTV_ADF1_HUNDREDS_DOWN,
	RP_CMD_ACTV_ADF1_HUNDREDS_UP,
	RP_CMD_ACTV_ADF1_TENS_DOWN,
	RP_CMD_ACTV_ADF1_TENS_UP,
	RP_CMD_ACTV_ADF1_ONES_DOWN,
	RP_CMD_ACTV_ADF1_ONES_UP,
	RP_CMD_ACTV_ADF2_HUNDREDS_DOWN,
	RP_CMD_ACTV_ADF2_HUNDREDS_UP,
	RP_CMD_ACTV_ADF2_TENS_DOWN,
	RP_CMD_ACTV_ADF2_TENS_UP,
	RP_CMD_ACTV_ADF2_ONES_DOWN,
	RP_CMD_ACTV_ADF2_ONES_UP,
	RP_CMD_XPDR_THOUSANDS_DOWN,
	RP_CMD_XPDR_THOUSANDS_UP,
	RP_CMD_XPDR_HUNDREDS_DOWN,
	RP_CMD_XPDR_HUNDREDS_UP,
	RP_CMD_XPDR_TENS_DOWN,
	RP_CMD_XPDR_TENS_UP,
	RP_CMD_XPDR_ONES_DOWN,
	RP_CMD_XPDR_ONES_UP

};

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
static struct rp_thread_data *gPtrThreadData;
static unsigned char buf[RP_IN_BUF_SIZE];
static unsigned char writeBuf[RP_OUT_BUF_SIZE];
static unsigned char writeBufPrev[RP_OUT_BUF_SIZE];
static char tmp[100];
static uint32_t gUpperKnob = 0;
static uint32_t gLowerKnob = 0;


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

XPLMCommandRef gRpStdbyCOM2FineDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2FineUpCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2CoarseDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2CoarseUpCmdRef = NULL;
XPLMCommandRef gRpActvCOM2FineDownCmdRef = NULL;
XPLMCommandRef gRpActvCOM2FineUpCmdRef = NULL;
XPLMCommandRef gRpActvCOM2CoarseDownCmdRef = NULL;
XPLMCommandRef gRpActvCOM2CoarseUpCmdRef = NULL;
XPLMCommandRef gRpCOM2StbyFlipCmdRef = NULL;

XPLMCommandRef gRpStdbyNAV1FineDownCmdRef = NULL;
XPLMCommandRef gRpStdbyNAV1FineUpCmdRef = NULL;
XPLMCommandRef gRpStdbyNAV1CoarseDownCmdRef = NULL;
XPLMCommandRef gRpStdbyNAV1CoarseUpCmdRef = NULL;
XPLMCommandRef gRpActvNAV1FineDownCmdRef = NULL;
XPLMCommandRef gRpActvNAV1FineUpCmdRef = NULL;
XPLMCommandRef gRpActvNAV1CoarseDownCmdRef = NULL;
XPLMCommandRef gRpActvNAV1CoarseUpCmdRef = NULL;
XPLMCommandRef gRpNAV1StbyFlipCmdRef = NULL;

XPLMCommandRef gRpStdbyNAV2FineDownCmdRef = NULL;
XPLMCommandRef gRpStdbyNAV2FineUpCmdRef = NULL;
XPLMCommandRef gRpStdbyNAV2CoarseDownCmdRef = NULL;
XPLMCommandRef gRpStdbyNAV2CoarseUpCmdRef = NULL;
XPLMCommandRef gRpActvNAV2FineDownCmdRef = NULL;
XPLMCommandRef gRpActvNAV2FineUpCmdRef = NULL;
XPLMCommandRef gRpActvNAV2CoarseDownCmdRef = NULL;
XPLMCommandRef gRpActvNAV2CoarseUpCmdRef = NULL;
XPLMCommandRef gRpNAV2StbyFlipCmdRef = NULL;

XPLMCommandRef gRpActvADF1HundredsUpCmdRef = NULL;
XPLMCommandRef gRpActvADF1HundredsDownCmdRef = NULL;
XPLMCommandRef gRpActvADF1TensUpCmdRef = NULL;
XPLMCommandRef gRpActvADF1TensDownCmdRef = NULL;
XPLMCommandRef gRpActvADF1OnesUpCmdRef = NULL;
XPLMCommandRef gRpActvADF1OnesDownCmdRef = NULL;
XPLMCommandRef gRpADF1StbyFlipCmdRef = NULL;

XPLMCommandRef gRpActvADF2HundredsUpCmdRef = NULL;
XPLMCommandRef gRpActvADF2HundredsDownCmdRef = NULL;
XPLMCommandRef gRpActvADF2TensUpCmdRef = NULL;
XPLMCommandRef gRpActvADF2TensDownCmdRef = NULL;
XPLMCommandRef gRpActvADF2OnesUpCmdRef = NULL;
XPLMCommandRef gRpActvADF2OnesDownCmdRef = NULL;
XPLMCommandRef gRpADF2StbyFlipCmdRef = NULL;

XPLMCommandRef gRpXpdrThousandsUpCmdRef = NULL;
XPLMCommandRef gRpXpdrThousandsDownCmdRef = NULL;
XPLMCommandRef gRpXpdrHundredsUpCmdRef = NULL;
XPLMCommandRef gRpXpdrHundredsDownCmdRef = NULL;
XPLMCommandRef gRpXpdrTensUpCmdRef = NULL;
XPLMCommandRef gRpXpdrTensDownCmdRef = NULL;
XPLMCommandRef gRpXpdrOnesUpCmdRef = NULL;
XPLMCommandRef gRpXpdrOnesDownCmdRef = NULL;

/* RADIO PANEL Data Refs */
XPLMDataRef gRpCOM1FreqHzDataRef = NULL;
XPLMDataRef gRpCOM1StdbyFreqHzDataRef = NULL;

XPLMDataRef gRpCOM2FreqHzDataRef = NULL;
XPLMDataRef gRpCOM2StdbyFreqHzDataRef = NULL;

XPLMDataRef gRpNAV1FreqHzDataRef = NULL;
XPLMDataRef gRpNAV1StdbyFreqHzDataRef = NULL;

XPLMDataRef gRpNAV2FreqHzDataRef = NULL;
XPLMDataRef gRpNAV2StdbyFreqHzDataRef = NULL;

XPLMDataRef gRpXpdrCodeDataRef = NULL;
XPLMDataRef gRpADF1FreqHzDataRef = NULL;
XPLMDataRef gRpADF1StdbyFreqHzDataRef = NULL;
XPLMDataRef gRpADF2FreqHzDataRef = NULL;
XPLMDataRef gRpADF2StdbyFreqHzDataRef = NULL;
XPLMDataRef gRpNAV1DmeDistanceNmDataRef = NULL;
XPLMDataRef gRpNAV1DmeSpeedKtsDataRef = NULL;
XPLMDataRef gRpNAV2DmeDistanceNmDataRef = NULL;
XPLMDataRef gRpNAV2DmeSpeedKtsDataRef = NULL;
XPLMDataRef gRpDmeSlaveSourceDataRef = NULL;

/* RADIO PANEL Global variables */
uint32_t gRpCOM1StbyFreq = 0;
uint32_t gRpCOM1Freq = 0;

uint32_t gRpCOM2StbyFreq = 0;
uint32_t gRpCOM2Freq = 0;

uint32_t gRpNAV1StbyFreq = 0;
uint32_t gRpNAV1Freq = 0;

uint32_t gRpNAV2StbyFreq = 0;
uint32_t gRpNAV2Freq = 0;

uint32_t gRpXpdrCode = 0;
uint32_t gRpADF1FreqHz = 0;
uint32_t gRpADF1StdbyFreqHz = 0;
uint32_t gRpADF2FreqHz = 0;
uint32_t gRpADF2StdbyFreqHz = 0;
uint32_t gRpNAV1DmeDistanceNm = 0;
uint32_t gRpNAV1DmeSpeedKts = 0;
uint32_t gRpNAV2DmeDistanceNm = 0;
uint32_t gRpNAV2DmeSpeedKts = 0;
uint32_t gRpDmeSlaveSource = 0;


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
			gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
			gRpCOM1Freq = (XPLMGetDatai(gRpCOM1FreqHzDataRef));
			break;
		case RP_CMD_ACTV_COM1_FINE_DOWN:
		case RP_CMD_ACTV_COM1_FINE_UP:
		case RP_CMD_ACTV_COM1_COARSE_DOWN:
		case RP_CMD_ACTV_COM1_COARSE_UP:
			gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
			gRpCOM1Freq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
			break;
		case RP_CMD_COM1_STANDBY_FLIP:
			gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1FreqHzDataRef));
			gRpCOM1Freq = (XPLMGetDatai(gRpCOM1FreqHzDataRef));
			break;
		case RP_CMD_STDBY_COM2_FINE_DOWN:
		case RP_CMD_STDBY_COM2_FINE_UP:
		case RP_CMD_STDBY_COM2_COARSE_DOWN:
		case RP_CMD_STDBY_COM2_COARSE_UP:
			gRpCOM2StbyFreq = (XPLMGetDatai(gRpCOM2StdbyFreqHzDataRef));
			gRpCOM2Freq = (XPLMGetDatai(gRpCOM2FreqHzDataRef));
			break;
		case RP_CMD_ACTV_COM2_FINE_DOWN:
		case RP_CMD_ACTV_COM2_FINE_UP:
		case RP_CMD_ACTV_COM2_COARSE_DOWN:
		case RP_CMD_ACTV_COM2_COARSE_UP:
			gRpCOM2StbyFreq = (XPLMGetDatai(gRpCOM2StdbyFreqHzDataRef));
			gRpCOM2Freq = (XPLMGetDatai(gRpCOM2StdbyFreqHzDataRef));
			break;
		case RP_CMD_COM2_STANDBY_FLIP:
			gRpCOM2StbyFreq = (XPLMGetDatai(gRpCOM2FreqHzDataRef));
			gRpCOM2Freq = (XPLMGetDatai(gRpCOM2FreqHzDataRef));
			break;
		case RP_CMD_STDBY_NAV1_FINE_DOWN:
		case RP_CMD_STDBY_NAV1_FINE_UP:
		case RP_CMD_STDBY_NAV1_COARSE_DOWN:
		case RP_CMD_STDBY_NAV1_COARSE_UP:
			gRpNAV1StbyFreq = (XPLMGetDatai(gRpNAV1StdbyFreqHzDataRef));
			gRpNAV1Freq = (XPLMGetDatai(gRpNAV1FreqHzDataRef));
			break;
		case RP_CMD_ACTV_NAV1_FINE_DOWN:
		case RP_CMD_ACTV_NAV1_FINE_UP:
		case RP_CMD_ACTV_NAV1_COARSE_DOWN:
		case RP_CMD_ACTV_NAV1_COARSE_UP:
			gRpNAV1StbyFreq = (XPLMGetDatai(gRpNAV1StdbyFreqHzDataRef));
			gRpNAV1Freq = (XPLMGetDatai(gRpNAV1StdbyFreqHzDataRef));
			break;
		case RP_CMD_NAV1_STANDBY_FLIP:
			gRpNAV1StbyFreq = (XPLMGetDatai(gRpNAV1FreqHzDataRef));
			gRpNAV1Freq = (XPLMGetDatai(gRpNAV1FreqHzDataRef));
			break;
		case RP_CMD_STDBY_NAV2_FINE_DOWN:
		case RP_CMD_STDBY_NAV2_FINE_UP:
		case RP_CMD_STDBY_NAV2_COARSE_DOWN:
		case RP_CMD_STDBY_NAV2_COARSE_UP:
			gRpNAV2StbyFreq = (XPLMGetDatai(gRpNAV2StdbyFreqHzDataRef));
			gRpNAV2Freq = (XPLMGetDatai(gRpNAV2FreqHzDataRef));
			break;
		case RP_CMD_ACTV_NAV2_FINE_DOWN:
		case RP_CMD_ACTV_NAV2_FINE_UP:
		case RP_CMD_ACTV_NAV2_COARSE_DOWN:
		case RP_CMD_ACTV_NAV2_COARSE_UP:
			gRpNAV2StbyFreq = (XPLMGetDatai(gRpNAV2StdbyFreqHzDataRef));
			gRpNAV2Freq = (XPLMGetDatai(gRpNAV2StdbyFreqHzDataRef));
			break;
		case RP_CMD_NAV2_STANDBY_FLIP:
			gRpNAV2StbyFreq = (XPLMGetDatai(gRpNAV2FreqHzDataRef));
			gRpNAV2Freq = (XPLMGetDatai(gRpNAV2FreqHzDataRef));
			break;
		case RP_CMD_ACTV_ADF1_HUNDREDS_DOWN:
		case RP_CMD_ACTV_ADF1_HUNDREDS_UP:
		case RP_CMD_ACTV_ADF1_TENS_DOWN:
		case RP_CMD_ACTV_ADF1_TENS_UP:
		case RP_CMD_ACTV_ADF1_ONES_DOWN:
		case RP_CMD_ACTV_ADF1_ONES_UP:
			gRpADF1FreqHz = (XPLMGetDatai(gRpADF1FreqHzDataRef));
			gRpADF1StdbyFreqHz = (XPLMGetDatai(gRpADF1StdbyFreqHzDataRef));
			break;
		case RP_CMD_ACTV_ADF2_HUNDREDS_DOWN:
		case RP_CMD_ACTV_ADF2_HUNDREDS_UP:
		case RP_CMD_ACTV_ADF2_TENS_DOWN:
		case RP_CMD_ACTV_ADF2_TENS_UP:
		case RP_CMD_ACTV_ADF2_ONES_DOWN:
		case RP_CMD_ACTV_ADF2_ONES_UP:
			gRpADF2FreqHz = (XPLMGetDatai(gRpADF2FreqHzDataRef));
			gRpADF2StdbyFreqHz = (XPLMGetDatai(gRpADF2StdbyFreqHzDataRef));
			break;
		case RP_CMD_XPDR_THOUSANDS_DOWN:
		case RP_CMD_XPDR_THOUSANDS_UP:
		case RP_CMD_XPDR_HUNDREDS_DOWN:
		case RP_CMD_XPDR_HUNDREDS_UP:
		case RP_CMD_XPDR_TENS_DOWN:
		case RP_CMD_XPDR_TENS_UP:
		case RP_CMD_XPDR_ONES_DOWN:
		case RP_CMD_XPDR_ONES_UP:
			gRpXpdrCode = (XPLMGetDatai(gRpXpdrCodeDataRef));
			break;
		default:
			break;
 }

 return status;
}

int rp_has_changed(unsigned char a[], unsigned char b[]) {
	int i = 0;
	for (i = 0; i < RP_OUT_BUF_SIZE; i++) {
		if (a[i] != b[i]) {
			return 1;
		}
	}
	return 0;
}

inline void rp_upper_led_update(uint32_t x, uint32_t y, uint32_t upperDecimal, uint32_t upperPos1, uint32_t upperPos2,
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

void rp_process_coarse_right(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x000001:
	case 0x000080:
		XPLMCommandOnce(gRpStdbyCOM1CoarseUpCmdRef);
		break;
	case 0x000002:
	case 0x000100:
		XPLMCommandOnce(gRpStdbyCOM2CoarseUpCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gRpStdbyNAV1CoarseUpCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gRpStdbyNAV2CoarseUpCmdRef);
		break;
	case 0x000010:
		XPLMCommandOnce(gRpActvADF1HundredsUpCmdRef);
		break;
	case 0x000800:
		XPLMCommandOnce(gRpActvADF2HundredsUpCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		//XPLMCommandOnce(gRpStdbyDMECoarseUpCmdRef);
		break;
	case 0x000040:
	case 0x002000:
		if (((gRpXpdrCode / 100) % 10) == 7) {
			XPLMCommandOnce(gRpXpdrThousandsUpCmdRef);
		}
		XPLMCommandOnce(gRpXpdrHundredsUpCmdRef);
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
		XPLMCommandOnce(gRpStdbyCOM2CoarseDownCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gRpStdbyNAV1CoarseDownCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gRpStdbyNAV2CoarseDownCmdRef);
		break;
	case 0x000010:
		XPLMCommandOnce(gRpActvADF1HundredsDownCmdRef);
		break;
	case 0x000800:
		XPLMCommandOnce(gRpActvADF2HundredsDownCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		if (((gRpXpdrCode / 100) % 10) == 0) {
			XPLMCommandOnce(gRpXpdrThousandsDownCmdRef);
		}
		XPLMCommandOnce(gRpXpdrHundredsDownCmdRef);
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
		XPLMCommandOnce(gRpStdbyCOM2FineUpCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gRpStdbyNAV1FineUpCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gRpStdbyNAV2FineUpCmdRef);
		break;
	case 0x000010:
		if ((gRpADF1StdbyFreqHz % 10) == 9) {
			XPLMCommandOnce(gRpActvADF1TensUpCmdRef);
		}
		XPLMCommandOnce(gRpActvADF1OnesUpCmdRef);
		break;
	case 0x000800:
		if ((gRpADF2StdbyFreqHz % 10) == 9) {
			XPLMCommandOnce(gRpActvADF2TensUpCmdRef);
		}
		XPLMCommandOnce(gRpActvADF2OnesUpCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		if ((gRpXpdrCode % 10) == 7) {
			XPLMCommandOnce(gRpXpdrTensUpCmdRef);
		}
		XPLMCommandOnce(gRpXpdrOnesUpCmdRef);
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
		XPLMCommandOnce(gRpStdbyCOM2FineDownCmdRef);
		break;
	case 0x000004:
	case 0x000200:
		XPLMCommandOnce(gRpStdbyNAV1FineDownCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gRpStdbyNAV2FineDownCmdRef);
		break;
	case 0x000010:
		if ((gRpADF1StdbyFreqHz % 10) == 0) {
			XPLMCommandOnce(gRpActvADF1TensDownCmdRef);
		}
		XPLMCommandOnce(gRpActvADF1OnesDownCmdRef);
		break;
	case 0x000800:
		if ((gRpADF2StdbyFreqHz % 10) == 0) {
			XPLMCommandOnce(gRpActvADF2TensDownCmdRef);
		}
		XPLMCommandOnce(gRpActvADF2OnesDownCmdRef);
		break;
	case 0x000020:
	case 0x001000:
		break;
	case 0x000040:
	case 0x002000:
		if ((gRpXpdrCode % 10) == 0) {
			XPLMCommandOnce(gRpXpdrTensDownCmdRef);
		}
		XPLMCommandOnce(gRpXpdrOnesDownCmdRef);
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
		XPLMCommandOnce(gRpNAV1StbyFlipCmdRef);
		break;
	case 0x000008:
	case 0x000400:
		XPLMCommandOnce(gRpNAV2StbyFlipCmdRef);
		break;
	case 0x000010:
		XPLMCommandOnce(gRpADF1StbyFlipCmdRef);
		break;
	case 0x000800:
		XPLMCommandOnce(gRpADF2StbyFlipCmdRef);
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
    //sprintf(tmp, "-> CP: rp_controller.rp_process: msg: %d\n", msg);
	//XPLMDebugString(tmp);
	int res = 0;
    gUpperKnob = msg & RP_READ_UPPER_KNOB_MODE_MASK;
    gLowerKnob = msg & RP_READ_LOWER_KNOB_MODE_MASK;
    uint32_t upperFineTuning = msg & RP_READ_UPPER_FINE_TUNING_MASK;
    uint32_t upperCoarseTuning = msg & RP_READ_UPPER_COARSE_TUNING_MASK;
    uint32_t lowerFineTuning = msg & RP_READ_LOWER_FINE_TUNING_MASK;
    uint32_t lowerCoarseTuning = msg & RP_READ_LOWER_COARSE_TUNING_MASK;
    uint32_t upperStby = msg & RP_READ_UPPER_ACT_STBY;
    uint32_t lowerStby = msg & RP_READ_LOWER_ACT_STBY;

    if (upperCoarseTuning || upperFineTuning) {
		if (upperCoarseTuning == RP_READ_UPPER_COARSE_RIGHT) {
			rp_process_coarse_right(gUpperKnob);
		} else if (upperCoarseTuning == RP_READ_UPPER_COARSE_LEFT) {
			rp_process_coarse_left(gUpperKnob);
		} else if (upperFineTuning == RP_READ_UPPER_FINE_RIGHT) {
			rp_process_fine_right(gUpperKnob);
		} else if (upperFineTuning == RP_READ_UPPER_FINE_LEFT) {
			rp_process_fine_left(gUpperKnob);
		}
    }
    if (upperStby) {
    	rp_process_switch(gUpperKnob);
    }
    if (lowerCoarseTuning || lowerFineTuning) {
		if (lowerCoarseTuning == RP_READ_LOWER_COARSE_RIGHT) {
			rp_process_coarse_right(gLowerKnob);
		} else if (lowerCoarseTuning == RP_READ_LOWER_COARSE_LEFT) {
			rp_process_coarse_left(gLowerKnob);
		} else if (lowerFineTuning == RP_READ_LOWER_FINE_RIGHT) {
			rp_process_fine_right(gLowerKnob);
		} else if (lowerFineTuning == RP_READ_LOWER_FINE_LEFT) {
			rp_process_fine_left(gLowerKnob);
		}
    }
    if (lowerStby) {
    	rp_process_switch(gLowerKnob);
    }
    return res;
}

void rp_update_datarefs() {
    gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
    gRpCOM1Freq = (XPLMGetDatai(gRpCOM1FreqHzDataRef));

    gRpCOM2StbyFreq = (XPLMGetDatai(gRpCOM2StdbyFreqHzDataRef));
    gRpCOM2Freq = (XPLMGetDatai(gRpCOM2FreqHzDataRef));

    gRpNAV1StbyFreq = (XPLMGetDatai(gRpNAV1StdbyFreqHzDataRef));
    gRpNAV1Freq = (XPLMGetDatai(gRpNAV1FreqHzDataRef));

    gRpNAV2StbyFreq = (XPLMGetDatai(gRpNAV2StdbyFreqHzDataRef));
    gRpNAV2Freq = (XPLMGetDatai(gRpNAV2FreqHzDataRef));

    gRpXpdrCode = (XPLMGetDatai(gRpXpdrCodeDataRef));
    gRpADF1FreqHz = (XPLMGetDatai(gRpADF1FreqHzDataRef));
    gRpADF1StdbyFreqHz = (XPLMGetDatai(gRpADF1StdbyFreqHzDataRef));
    gRpADF2FreqHz = (XPLMGetDatai(gRpADF2FreqHzDataRef));
    gRpADF2StdbyFreqHz = (XPLMGetDatai(gRpADF2StdbyFreqHzDataRef));
    gRpNAV1DmeDistanceNm = (XPLMGetDatai(gRpNAV1DmeDistanceNmDataRef));
    gRpNAV1DmeSpeedKts = (XPLMGetDatai(gRpNAV1DmeSpeedKtsDataRef));
    gRpNAV2DmeDistanceNm = (XPLMGetDatai(gRpNAV2DmeDistanceNmDataRef));
    gRpNAV2DmeSpeedKts = (XPLMGetDatai(gRpNAV2DmeSpeedKtsDataRef));
    gRpDmeSlaveSource = (XPLMGetDatai(gRpDmeSlaveSourceDataRef));

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

	gRpStdbyCOM2FineDownCmdRef         = XPLMFindCommand(sRP_STDBY_COM2_FINE_DOWN_CR);
	gRpStdbyCOM2FineUpCmdRef           = XPLMFindCommand(sRP_STDBY_COM2_FINE_UP_CR);
	gRpStdbyCOM2CoarseDownCmdRef       = XPLMFindCommand(sRP_STDBY_COM2_COARSE_DOWN_CR);
	gRpStdbyCOM2CoarseUpCmdRef         = XPLMFindCommand(sRP_STDBY_COM2_COARSE_UP_CR);
	gRpActvCOM2FineDownCmdRef         = XPLMFindCommand(sRP_ACTV_COM2_FINE_DOWN_CR);
	gRpActvCOM2FineUpCmdRef           = XPLMFindCommand(sRP_ACTV_COM2_FINE_UP_CR);
	gRpActvCOM2CoarseDownCmdRef       = XPLMFindCommand(sRP_ACTV_COM2_COARSE_DOWN_CR);
	gRpActvCOM2CoarseUpCmdRef         = XPLMFindCommand(sRP_ACTV_COM2_COARSE_UP_CR);
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

    gRpCOM1FreqHzDataRef        = XPLMFindDataRef(sRP_COM1_FREQ_HZ_DR);
    gRpCOM1StdbyFreqHzDataRef   = XPLMFindDataRef(sRP_COM1_STDBY_FREQ_HZ_DR);

    gRpCOM1StbyFreq = (XPLMGetDatai(gRpCOM1StdbyFreqHzDataRef));
    gRpCOM1Freq = (XPLMGetDatai(gRpCOM1FreqHzDataRef));

    XPLMRegisterCommandHandler(gRpStdbyCOM2FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM2_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyCOM2FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM2_FINE_UP);
    XPLMRegisterCommandHandler(gRpStdbyCOM2CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyCOM2CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_COM2_COARSE_UP);
    XPLMRegisterCommandHandler(gRpActvCOM2FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM2_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpActvCOM2FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM2_FINE_UP);
    XPLMRegisterCommandHandler(gRpActvCOM2CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpActvCOM2CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_COM2_COARSE_UP);
    XPLMRegisterCommandHandler(gRpCOM2StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_COM2_STANDBY_FLIP);

    gRpCOM2FreqHzDataRef        = XPLMFindDataRef(sRP_COM2_FREQ_HZ_DR);
    gRpCOM2StdbyFreqHzDataRef   = XPLMFindDataRef(sRP_COM2_STDBY_FREQ_HZ_DR);

    gRpCOM2StbyFreq = (XPLMGetDatai(gRpCOM2StdbyFreqHzDataRef));
    gRpCOM2Freq = (XPLMGetDatai(gRpCOM2FreqHzDataRef));

	gRpStdbyNAV1FineDownCmdRef         = XPLMFindCommand(sRP_STDBY_NAV1_FINE_DOWN_CR);
	gRpStdbyNAV1FineUpCmdRef           = XPLMFindCommand(sRP_STDBY_NAV1_FINE_UP_CR);
	gRpStdbyNAV1CoarseDownCmdRef       = XPLMFindCommand(sRP_STDBY_NAV1_COARSE_DOWN_CR);
	gRpStdbyNAV1CoarseUpCmdRef         = XPLMFindCommand(sRP_STDBY_NAV1_COARSE_UP_CR);
	gRpActvNAV1FineDownCmdRef         = XPLMFindCommand(sRP_ACTV_NAV1_FINE_DOWN_CR);
	gRpActvNAV1FineUpCmdRef           = XPLMFindCommand(sRP_ACTV_NAV1_FINE_UP_CR);
	gRpActvNAV1CoarseDownCmdRef       = XPLMFindCommand(sRP_ACTV_NAV1_COARSE_DOWN_CR);
	gRpActvNAV1CoarseUpCmdRef         = XPLMFindCommand(sRP_ACTV_NAV1_COARSE_UP_CR);
	gRpNAV1StbyFlipCmdRef              = XPLMFindCommand(sRP_NAV1_STBY_FLIP_CR);

	gRpStdbyNAV2FineDownCmdRef         = XPLMFindCommand(sRP_STDBY_NAV2_FINE_DOWN_CR);
	gRpStdbyNAV2FineUpCmdRef           = XPLMFindCommand(sRP_STDBY_NAV2_FINE_UP_CR);
	gRpStdbyNAV2CoarseDownCmdRef       = XPLMFindCommand(sRP_STDBY_NAV2_COARSE_DOWN_CR);
	gRpStdbyNAV2CoarseUpCmdRef         = XPLMFindCommand(sRP_STDBY_NAV2_COARSE_UP_CR);
	gRpActvNAV2FineDownCmdRef         = XPLMFindCommand(sRP_ACTV_NAV2_FINE_DOWN_CR);
	gRpActvNAV2FineUpCmdRef           = XPLMFindCommand(sRP_ACTV_NAV2_FINE_UP_CR);
	gRpActvNAV2CoarseDownCmdRef       = XPLMFindCommand(sRP_ACTV_NAV2_COARSE_DOWN_CR);
	gRpActvNAV2CoarseUpCmdRef         = XPLMFindCommand(sRP_ACTV_NAV2_COARSE_UP_CR);
	gRpNAV2StbyFlipCmdRef              = XPLMFindCommand(sRP_NAV2_STBY_FLIP_CR);
	gRpADF1StbyFlipCmdRef              = XPLMFindCommand(sRP_ADF1_STBY_FLIP_CR);
	gRpADF2StbyFlipCmdRef              = XPLMFindCommand(sRP_ADF2_STBY_FLIP_CR);

    XPLMRegisterCommandHandler(gRpStdbyNAV1FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV1_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyNAV1FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV1_FINE_UP);
    XPLMRegisterCommandHandler(gRpStdbyNAV1CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyNAV1CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV1_COARSE_UP);
    XPLMRegisterCommandHandler(gRpActvNAV1FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV1_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpActvNAV1FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV1_FINE_UP);
    XPLMRegisterCommandHandler(gRpActvNAV1CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV1_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpActvNAV1CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV1_COARSE_UP);
    XPLMRegisterCommandHandler(gRpNAV1StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_NAV1_STANDBY_FLIP);

    gRpNAV1FreqHzDataRef        = XPLMFindDataRef(sRP_NAV1_FREQ_HZ_DR);
    gRpNAV1StdbyFreqHzDataRef   = XPLMFindDataRef(sRP_NAV1_STDBY_FREQ_HZ_DR);

    gRpNAV1StbyFreq = (XPLMGetDatai(gRpNAV1StdbyFreqHzDataRef));
    gRpNAV1Freq = (XPLMGetDatai(gRpNAV1FreqHzDataRef));

    XPLMRegisterCommandHandler(gRpStdbyNAV2FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV2_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyNAV2FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV2_FINE_UP);
    XPLMRegisterCommandHandler(gRpStdbyNAV2CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpStdbyNAV2CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_STDBY_NAV2_COARSE_UP);
    XPLMRegisterCommandHandler(gRpActvNAV2FineDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV2_FINE_DOWN);
    XPLMRegisterCommandHandler(gRpActvNAV2FineUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV2_FINE_UP);
    XPLMRegisterCommandHandler(gRpActvNAV2CoarseDownCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV2_COARSE_DOWN);
    XPLMRegisterCommandHandler(gRpActvNAV2CoarseUpCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ACTV_NAV2_COARSE_UP);
    XPLMRegisterCommandHandler(gRpNAV2StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_NAV2_STANDBY_FLIP);
    XPLMRegisterCommandHandler(gRpADF1StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ADF1_STANDBY_FLIP);
    XPLMRegisterCommandHandler(gRpADF2StbyFlipCmdRef, RadioPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) RP_CMD_ADF2_STANDBY_FLIP);

    gRpNAV2FreqHzDataRef        = XPLMFindDataRef(sRP_NAV2_FREQ_HZ_DR);
    gRpNAV2StdbyFreqHzDataRef   = XPLMFindDataRef(sRP_NAV2_STDBY_FREQ_HZ_DR);

    gRpNAV2StbyFreq = (XPLMGetDatai(gRpNAV2StdbyFreqHzDataRef));
    gRpNAV2Freq = (XPLMGetDatai(gRpNAV2FreqHzDataRef));

    gRpActvADF1HundredsUpCmdRef = XPLMFindCommand(sRP_ACTV_ADF1_HUNDREDS_UP_CR);
    gRpActvADF1HundredsDownCmdRef = XPLMFindCommand(sRP_ACTV_ADF1_HUNDREDS_DOWN_CR);
    gRpActvADF1TensUpCmdRef = XPLMFindCommand(sRP_ACTV_ADF1_TENS_UP_CR);
    gRpActvADF1TensDownCmdRef = XPLMFindCommand(sRP_ACTV_ADF1_TENS_DOWN_CR);
    gRpActvADF1OnesUpCmdRef = XPLMFindCommand(sRP_ACTV_ADF1_ONES_UP_CR);
    gRpActvADF1OnesDownCmdRef = XPLMFindCommand(sRP_ACTV_ADF1_ONES_DOWN_CR);

    gRpActvADF2HundredsUpCmdRef = XPLMFindCommand(sRP_ACTV_ADF2_HUNDREDS_UP_CR);
    gRpActvADF2HundredsDownCmdRef = XPLMFindCommand(sRP_ACTV_ADF2_HUNDREDS_DOWN_CR);
    gRpActvADF2TensUpCmdRef = XPLMFindCommand(sRP_ACTV_ADF2_TENS_UP_CR);
    gRpActvADF2TensDownCmdRef = XPLMFindCommand(sRP_ACTV_ADF2_TENS_DOWN_CR);
    gRpActvADF2OnesUpCmdRef = XPLMFindCommand(sRP_ACTV_ADF2_ONES_UP_CR);
    gRpActvADF2OnesDownCmdRef = XPLMFindCommand(sRP_ACTV_ADF2_ONES_DOWN_CR);

    gRpXpdrThousandsUpCmdRef = XPLMFindCommand(sRP_XPDR_THOUSANDS_UP_CR);
    gRpXpdrThousandsDownCmdRef = XPLMFindCommand(sRP_XPDR_THOUSANDS_DOWN_CR);
    gRpXpdrHundredsUpCmdRef = XPLMFindCommand(sRP_XPDR_HUNDREDS_UP_CR);
    gRpXpdrHundredsDownCmdRef = XPLMFindCommand(sRP_XPDR_HUNDREDS_DOWN_CR);
    gRpXpdrTensUpCmdRef = XPLMFindCommand(sRP_XPDR_TENS_UP_CR);
    gRpXpdrTensDownCmdRef = XPLMFindCommand(sRP_XPDR_TENS_DOWN_CR);
    gRpXpdrOnesUpCmdRef = XPLMFindCommand(sRP_XPDR_ONES_UP_CR);
    gRpXpdrOnesDownCmdRef = XPLMFindCommand(sRP_XPDR_ONES_DOWN_CR);

    gRpXpdrCodeDataRef = XPLMFindDataRef(sRP_XPDR_CODE_DR);
    gRpADF1FreqHzDataRef = XPLMFindDataRef(sRP_ADF1_FREQ_HZ_DR);
    gRpADF1StdbyFreqHzDataRef = XPLMFindDataRef(sRP_ADF1_STDBY_FREQ_HZ_DR);
    gRpADF2FreqHzDataRef = XPLMFindDataRef(sRP_ADF2_FREQ_HZ_DR);
    gRpADF2StdbyFreqHzDataRef = XPLMFindDataRef(sRP_ADF2_STDBY_FREQ_HZ_DR);
    gRpNAV1DmeDistanceNmDataRef = XPLMFindDataRef(sRP_NAV1_DME_DISTANCE_NM_DR);
    gRpNAV1DmeSpeedKtsDataRef = XPLMFindDataRef(sRP_NAV1_DME_SPEED_KTS_DR);
    gRpNAV2DmeDistanceNmDataRef = XPLMFindDataRef(sRP_NAV2_DME_DISTANCE_NM_DR);
    gRpNAV2DmeSpeedKtsDataRef = XPLMFindDataRef(sRP_NAV2_DME_SPEED_KTS_DR);
    gRpDmeSlaveSourceDataRef = XPLMFindDataRef(sRP_DME_SLAVE_SOURCE_DR);

    gRpXpdrCode = (XPLMGetDatai(gRpXpdrCodeDataRef));
    gRpADF1FreqHz = (XPLMGetDatai(gRpADF1FreqHzDataRef));
    gRpADF1StdbyFreqHz = (XPLMGetDatai(gRpADF1StdbyFreqHzDataRef));
    gRpADF2FreqHz = (XPLMGetDatai(gRpADF2FreqHzDataRef));
    gRpADF2StdbyFreqHz = (XPLMGetDatai(gRpADF2StdbyFreqHzDataRef));
    gRpNAV1DmeDistanceNm = (XPLMGetDatai(gRpNAV1DmeDistanceNmDataRef));
    gRpNAV1DmeSpeedKts = (XPLMGetDatai(gRpNAV1DmeSpeedKtsDataRef));
    gRpNAV2DmeDistanceNm = (XPLMGetDatai(gRpNAV2DmeDistanceNmDataRef));
    gRpNAV2DmeSpeedKts = (XPLMGetDatai(gRpNAV2DmeSpeedKtsDataRef));
    gRpDmeSlaveSource = (XPLMGetDatai(gRpDmeSlaveSourceDataRef));
}

void rp_prepare_write_buffer(int i, int j) {
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
		tmp1 = dec2bcd(gRpCOM1Freq, 5);
		tmp2 = dec2bcd(gRpCOM1StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000002:
	case 0x000100:
		tmp1 = dec2bcd(gRpCOM2Freq, 5);
		tmp2 = dec2bcd(gRpCOM2StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000004:
	case 0x000200:
		tmp1 = dec2bcd(gRpNAV1Freq, 5);
		tmp2 = dec2bcd(gRpNAV1StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000008:
	case 0x000400:
		tmp1 = dec2bcd(gRpNAV2Freq, 5);
		tmp2 = dec2bcd(gRpNAV2StbyFreq, 5);
		upperDecimal = 0xD0;
		break;
	case 0x000010:
	case 0x000800:
		tmp1 = dec2bcd(gRpADF1FreqHz, 5);
		tmp2 = dec2bcd(gRpADF1StdbyFreqHz, 5);
		upperPos1 = 0xFF;
		upperPos2 = 0xFF;
		break;
	case 0x000020:
	case 0x001000:
		tmp1 = dec2bcd(gRpNAV1DmeDistanceNm, 5);
		tmp2 = dec2bcd(gRpNAV1DmeSpeedKts, 5);
		break;
	case 0x000040:
	case 0x002000:
		tmp1 = dec2bcd(gRpXpdrCode, 5);
		tmp2 = dec2bcd(gRpXpdrCode, 5);
		upperPos1 = 0xFF;
		break;
	default:
		break;
	}
	switch (gLowerKnob) {
	case 0x000001:
	case 0x000080:
		tmp3 = dec2bcd(gRpCOM1Freq, 5);
		tmp4 = dec2bcd(gRpCOM1StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000002:
	case 0x000100:
		tmp3 = dec2bcd(gRpCOM2Freq, 5);
		tmp4 = dec2bcd(gRpCOM2StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000004:
	case 0x000200:
		tmp3 = dec2bcd(gRpNAV1Freq, 5);
		tmp4 = dec2bcd(gRpNAV1StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000008:
	case 0x000400:
		tmp3 = dec2bcd(gRpNAV2Freq, 5);
		tmp4 = dec2bcd(gRpNAV2StbyFreq, 5);
		lowerDecimal = 0xD0;
		break;
	case 0x000010:
	case 0x000800:
		tmp3 = dec2bcd(gRpADF2FreqHz, 5);
		tmp4 = dec2bcd(gRpADF2StdbyFreqHz, 5);
		lowerPos1 = 0xFF;
		lowerPos2 = 0xFF;
		break;
	case 0x000020:
	case 0x001000:
		tmp3 = dec2bcd(gRpNAV2DmeDistanceNm, 5);
		tmp4 = dec2bcd(gRpNAV2DmeSpeedKts, 5);
//		tmp3 = dec2bcd(i % 100000, 5);
//		tmp4 = dec2bcd(j % 1000000, 5);
		break;
	case 0x000040:
	case 0x002000:
		tmp3 = dec2bcd(gRpXpdrCode, 5);
		tmp4 = dec2bcd(gRpXpdrCode, 5);
		lowerPos1 = 0xFF;
		break;
	default:
		break;
	}
	rp_upper_led_update(tmp1, tmp2, upperDecimal, upperPos1, upperPos2, tmp3, tmp4, lowerDecimal, lowerPos1, lowerPos2, writeBuf);
}


void *rpRun(void *ptr_thread_data) {
	int counter = 0;
	int counter2 = 0;
	int inReportBytesCount = 0;

#if IBM
		Sleep(SLEEP_TIME * 2);
#else
		usleep(SLEEP_TIME * 2);
#endif

	rp_init();

	gPtrThreadData = (struct rp_thread_data *) ptr_thread_data;

	int result = rp_panel_open();
	if (result < 0) {
		XPLMDebugString("-> CP: rp_controller.rpRun: shutdown thread.\n");
		pthread_exit(NULL);
		return 0;
	}
	XPLMDebugString("-> CP: rp_controller.rpRun: panel opened.\n");
	rp_panel_write(rp_blank_panel);
	inReportBytesCount = rp_panel_read_non_blocking(buf);
	// URB_FUNCTION_CLASS_INTERFACE request should be 0x01 instead of 0x09
	rp_panel_write_empty();
	inReportBytesCount = rp_panel_read_non_blocking(buf);
	rp_panel_write(rp_zero_panel);
	inReportBytesCount = rp_panel_read_non_blocking(buf);

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
			rp_prepare_write_buffer(counter, counter2);
			inReportBytesCount = rp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: rp_controller.rpRun: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				} else if (inReportBytesCount == 2) {
					counter2++;
					uint32_t msg = 0;
					msg += buf[1] << 16;
					msg += buf[0] << 8;
					rp_process(msg);
				} else {
					counter2++;
					uint32_t msg = 0;
					msg += buf[2] << 16;
					msg += buf[1] << 8;
					msg += buf[0];
					rp_process(msg);
				}
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// Update Panel. CRITICAL 20 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(50000, COUNTER2, loop_start_time)) {
			// Update local DataRefs.
			rp_update_datarefs();
			// update Panel.
			inReportBytesCount = rp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: rp_controller.rpRun: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				} else if (inReportBytesCount == 2) {
					counter2++;
					uint32_t msg = 0;
					msg += buf[1] << 16;
					msg += buf[0] << 8;
					rp_process(msg);
				} else {
					uint32_t msg = 0;
					msg += buf[2] << 16;
					msg += buf[1] << 8;
					msg += buf[0];
					rp_process(msg);
				}
			}
			if (rp_has_changed(writeBuf, writeBufPrev)) {
				int i = 0;
				for(i = 0; i < RP_OUT_BUF_SIZE; i++) {
					writeBufPrev[i] = writeBuf[i];
				}
				rp_panel_write(writeBuf);
			}
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= MAX_DELAY_TIME) {
			XPLMDebugString("-> CP: rp_controller.rpRun: CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		} else {
			//writeConsole(0, 0, "CPU LOAD OK.");
		}

		// wait 1 milliseconds
#if IBM
		Sleep(SLEEP_TIME * 2);
#else
		usleep(SLEEP_TIME * 2);
#endif
	}
	rp_panel_close();
#if IBM
		Sleep(SLEEP_TIME * 2);
#else
		usleep(SLEEP_TIME * 2);
#endif
	pthread_exit(NULL);
	return 0;
}

