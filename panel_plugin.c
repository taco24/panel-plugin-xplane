#if IBM
#include <windows.h>
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <pthread.h>
#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

#include "defs.h"
#include "panel_plugin.h"
#include "settings.h"
#include "properties.h"
#include "log.h"
#include "rp_controller.h"
#include "mp_controller.h"

// Radio Panel
#define sRP_STDBY_COM1_FINE_DOWN_CR       "sim/radios/stby_com1_fine_down"
#define sRP_STDBY_COM1_FINE_UP_CR         "sim/radios/stby_com1_fine_up"
#define sRP_STDBY_COM1_COARSE_DOWN_CR     "sim/radios/stby_com1_coarse_down"
#define sRP_STDBY_COM1_COARSE_UP_CR       "sim/radios/stby_com1_coarse_up"

#define sRP_STDBY_COM2_FINE_DOWN_CR       "sim/radios/stby_com2_fine_down"
#define sRP_STDBY_COM2_FINE_UP_CR         "sim/radios/stby_com2_fine_up"
#define sRP_STDBY_COM2_COARSE_DOWN_CR     "sim/radios/stby_com2_coarse_down"
#define sRP_STDBY_COM2_COARSE_UP_CR       "sim/radios/stby_com2_coarse_up"

#define sRP_COM1_STANDBY_FLIP_CR          "sim/radios/com1_standy_flip"
#define sRP_COM2_STANDBY_FLIP_CR          "sim/radios/com2_standy_flip"

#define sRP_COM1_FREQ_HZ_DR               "sim/cockpit/radios/com1_freq_hz"
#define sRP_COM2_FREQ_HZ_DR               "sim/cockpit/radios/com2_freq_hz"
#define sRP_COM1_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/com1_stdby_freq_hz"
#define sRP_COM2_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/com2_stdby_freq_hz"

#define CMD_HANDLER_PROLOG                (1)
#define CMD_HANDLER_EPILOG                (0)

int RadioPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void*             inRefcon);

float RadioPanelFlightLoopCallback(float inElapsedSinceLastCall,
                                   float inElapsedTimeSinceLastFlightLoop,
                                   int inCounter,
                                   void* inRefcon);

enum {
    PLUGIN_PLANE_ID = 0
};

// Radio panel
enum {
    RP_CMD_EAT_EVENT = 0,
    RP_CMD_PASS_EVENT = 1,
	RP_CMD_STDBY_COM1_FINE_DOWN,
	RP_CMD_STDBY_COM1_FINE_UP,
	RP_CMD_STDBY_COM1_COARSE_DOWN,
	RP_CMD_STDBY_COM1_COARSE_UP,
	RP_CMD_STDBY_COM2_FINE_DOWN,
	RP_CMD_STDBY_COM2_FINE_UP,
	RP_CMD_STDBY_COM2_COARSE_DOWN,
	RP_CMD_STDBY_COM2_COARSE_UP,
	RP_CMD_COM1_STANDBY_FLIP,
	RP_CMD_COM2_STANDBY_FLIP
};

// Flightloop Callback INterval
static const float FL_CB_INTERVAL = -1.0;

/* RADIO PANEL Command Refs */
XPLMCommandRef gRpStdbyCOM1FineDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM1FineUpCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM1CoarseDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM1CoarseUpCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2FineDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2FineUpCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2CoarseDownCmdRef = NULL;
XPLMCommandRef gRpStdbyCOM2CoarseUpCmdRef = NULL;

/* RADIO PANEL Data Refs */
XPLMDataRef gRpCOM1FreqHzDataRef = NULL;
XPLMDataRef gRpCOM2FreqHzDataRef = NULL;
XPLMDataRef gRpCOM1StdbyFreqHzDataRef = NULL;
XPLMDataRef gRpCOM2StdbyFreqHzDataRef = NULL;


uint32_t gRpTuningThresh = 4;
uint32_t gRpUpperFineTuneUpCnt = 0;
uint32_t gRpUpperFineTuneDownCnt = 0;
uint32_t gRpUpperCoarseTuneUpCnt = 0;
uint32_t gRpUpperCoarseTuneDownCnt = 0;

pthread_t gRpController;
struct thread_data gRpThreadData;
pthread_t gRpThread;
int gRpThreadID = 1;
int gRpThreadReturnCode = 0;

pthread_t gMpController;
struct thread_data gMpThreadData;
pthread_t gMpThread;
int gMpThreadID = 2;
int gMpThreadReturnCode = 0;


int RadioPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void*             inRefcon) {
	return 1;
}

float RadioPanelFlightLoopCallback(float   inElapsedSinceLastCall,
                                   float   inElapsedTimeSinceLastFlightLoop,
                                   int     inCounter,
                                   void*   inRefcon) {
    return 1.0;
}

PLUGIN_API int XPluginStart(char * outName, char * outSig, char * outDesc) {
	/* First we must fill in the passed in buffers to describe our
	 * plugin to the plugin-system. */

	strcpy(outName, "X-Plane Panel Controller");
	strcpy(outSig, "colomboard.flapindicator");
	strcpy(outDesc, "colomboard 12.01.22");

    XPLMRegisterFlightLoopCallback(RadioPanelFlightLoopCallback, FL_CB_INTERVAL, NULL);

    XPLMDebugString("-> CP: XPluginStart\n");
	return 1;
}

PLUGIN_API int XPluginEnable(void) {
	XPLMDebugString("-> CP: XPluginEnable\n");
	// Radio Panel
	gRpThreadData.thread_id = gRpThreadID;
	gRpThreadData.stop = 0;
	gRpThreadReturnCode = pthread_create(&gRpThread, NULL, run,
			(void *) &gRpThreadData);
	if (gRpThreadReturnCode) {
		XPLMDebugString("-> CP: XPluginEnable: Could not start RpThread.\n");
		return 0;
	} else {
		XPLMDebugString("-> CP: XPluginEnable: RpThread started.\n");
	}

	// Multi Panel
	gMpThreadData.thread_id = gMpThreadID;
	gMpThreadData.stop = 0;
	gMpThreadReturnCode = pthread_create(&gMpThread, NULL, mpRun,
			(void *) &gMpThreadData);
	if (gMpThreadReturnCode) {
		XPLMDebugString("-> CP: XPluginEnable: Could not start MpThread.\n");
		return 0;
	} else {
		XPLMDebugString("-> CP: XPluginEnable: MpThread started.\n");
	}

	return 1;
}

PLUGIN_API void XPluginDisable(void) {
	XPLMDebugString("-> CP: XPluginDisable\n");
	gRpThreadData.stop = 1;
	gMpThreadData.stop = 1;
#if IBM
	Sleep(500);
#else
	sleep(1);
#endif
}

PLUGIN_API void XPluginStop(void) {
	XPLMUnregisterFlightLoopCallback(RadioPanelFlightLoopCallback, NULL);
	XPLMDebugString("-> CP: XPluginStop\n");
}



PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, long inMsg, void* inParam) {
    if (inFrom == XPLM_PLUGIN_XPLANE) {
    	int inparam = (int)(inParam);
        switch (inMsg) {
        case XPLM_MSG_PLANE_LOADED:
            if (inparam != PLUGIN_PLANE_ID) {
                break;
            }
            XPLMDebugString("-> CP: XPluginReceiveMessage XPLM_MSG_PLANE_LOADED\n");
            break;
        case XPLM_MSG_AIRPORT_LOADED:
        	XPLMDebugString("-> CP: XPluginReceiveMessage XPLM_MSG_AIRPORT_LOADED\n");
            break;
        case XPLM_MSG_SCENERY_LOADED:
        	XPLMDebugString("-> CP: XPluginReceiveMessage XPLM_MSG_SCENERY_LOADED\n");
            break;
        case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
        	XPLMDebugString("-> CP: XPluginReceiveMessage XPLM_MSG_AIRPLANE_COUNT_CHANGED\n");
            break;
        case XPLM_MSG_PLANE_CRASHED:
            if (inparam != PLUGIN_PLANE_ID) {
                break;
            }
            XPLMDebugString("-> CP: XPluginReceiveMessage XPLM_MSG_PLANE_CRASHED\n");
            break;
        case XPLM_MSG_PLANE_UNLOADED:
            if (inparam != PLUGIN_PLANE_ID) {
                break;
            }
            XPLMDebugString("-> CP: XPluginReceiveMessage XPLM_MSG_PLANE_UNLOADED\n");
            break;
        default:
        	XPLMDebugString("-> CP: XPluginReceiveMessage default\n");
            break;
        } // switch (inMsg)
    } // if (inFrom == XPLM_PLUGIN_XPLANE)
}
