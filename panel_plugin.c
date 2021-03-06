#if IBM
#include <windows.h>
#else
#include <unistd.h>
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
#include "sp_controller.h"
#include "mcp_controller.h"
#include "cb_controller.h"

enum {
    PLUGIN_PLANE_ID = 0
};

// Flightloop Callback INterval
static const float FL_CB_INTERVAL = -1.0;

struct rp_thread_data gRpThreadData;
pthread_t gRpThread;
int gRpThreadID = 1;
int gRpThreadReturnCode = 0;

struct thread_data gMpThreadData;
pthread_t gMpThread;
int gMpThreadID = 2;
int gMpThreadReturnCode = 0;

struct thread_data gSpThreadData;
pthread_t gSpThread;
int gSpThreadID = 3;
int gSpThreadReturnCode = 0;

struct thread_data gMcpThreadData;
pthread_t gMcpThread;
int gMcpThreadID = 2;
int gMcpThreadReturnCode = 0;

struct thread_data gCbThreadData;
pthread_t gCbThread;
int gCbThreadID = 2;
int gCbThreadReturnCode = 0;

XPLMDataRef gPanelBatteryOnDataRef = NULL;
XPLMDataRef gPanelGeneratorOnDataRef = NULL;
XPLMDataRef gPanelAvionicsOnDataRef = NULL;
uint32_t gPanelBatteryOn = 0;
uint32_t gPanelGeneratorOn = 0;
uint32_t gPanelAvionicsOn = 0;

float PanelFlightLoopCallback(float   inElapsedSinceLastCall,
                                   float   inElapsedTimeSinceLastFlightLoop,
                                   int     inCounter,
                                   void*   inRefcon) {

	// trimwheel changes must be synchronized with OpenGL to prevent X-Plane crashes.
	mp_process_trimwheel();
	gPanelBatteryOn = (XPLMGetDatai(gPanelBatteryOnDataRef));
	gPanelGeneratorOn = (XPLMGetDatai(gPanelGeneratorOnDataRef));
	gPanelAvionicsOn = (XPLMGetDatai(gPanelAvionicsOnDataRef));
	if (gMpThreadData.stop == 0) {
		mp_process_power(gPanelBatteryOn, gPanelGeneratorOn, gPanelAvionicsOn);
	}
	if (gRpThreadData.stop == 0) {
		rp_process_power(gPanelBatteryOn, gPanelGeneratorOn, gPanelAvionicsOn);
	}
	if (gCbThreadData.stop == 0) {
		cb_process_power(gPanelBatteryOn, gPanelGeneratorOn, gPanelAvionicsOn);
	}
	return FL_CB_INTERVAL; // call again next loop;
}

// Callback for Error Tests
void	MyErrorCB(const char * msg)
{
	XPLMDebugString("================================================================\n");
	XPLMDebugString("-> CP ERROR: ");
	XPLMDebugString(msg);
	XPLMDebugString("\n");
	XPLMDebugString("----------------------------------------------------------------\n");
}

PLUGIN_API int XPluginStart(char * outName, char * outSig, char * outDesc) {
	/* First we must fill in the passed in buffers to describe our
	 * plugin to the plugin-system. */

	strcpy(outName, "X-Plane Panel Controller");
	strcpy(outSig, "panel.controller");
	strcpy(outDesc, "panel 12.05.15");

	// Register the callback for errors
	XPLMSetErrorCallback(MyErrorCB);

    XPLMRegisterFlightLoopCallback(PanelFlightLoopCallback, FL_CB_INTERVAL, NULL);

	XPLMDebugString("-> CP: XPluginStart\n");
	return 1;
}

PLUGIN_API int XPluginEnable(void) {
	XPLMDebugString("-> CP: XPluginEnable\n");

	// Radio Panel
	gRpThreadData.thread_id = gRpThreadID;
	gRpThreadData.stop = 0;
	gRpThreadReturnCode = pthread_create(&gRpThread, NULL, rpRun,
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

	// Switch Panel
	gSpThreadData.thread_id = gSpThreadID;
	gSpThreadData.stop = 0;
	gSpThreadReturnCode = pthread_create(&gSpThread, NULL, spRun,
			(void *) &gSpThreadData);
	if (gSpThreadReturnCode) {
		XPLMDebugString("-> CP: XPluginEnable: Could not start SpThread.\n");
		return 0;
	} else {
		XPLMDebugString("-> CP: XPluginEnable: SpThread started.\n");
	}

	// MCPPro Panel
	gMcpThreadData.thread_id = gMcpThreadID;
	gMcpThreadData.stop = 0;
	gMcpThreadReturnCode = pthread_create(&gMcpThread, NULL, mcpRun,
			(void *) &gMcpThreadData);
	if (gMcpThreadReturnCode) {
		XPLMDebugString("-> CP: XPluginEnable: Could not start McpThread.\n");
		return 0;
	} else {
		XPLMDebugString("-> CP: XPluginEnable: McpThread started.\n");
	}

	// Colomboard Panel
	gCbThreadData.thread_id = gCbThreadID;
	gCbThreadData.stop = 0;
	gCbThreadReturnCode = pthread_create(&gCbThread, NULL, cbRun,
			(void *) &gCbThreadData);
	if (gCbThreadReturnCode) {
		XPLMDebugString("-> CP: XPluginEnable: Could not start CbThread.\n");
		return 0;
	} else {
		XPLMDebugString("-> CP: XPluginEnable: CbThread started.\n");
	}

	gPanelBatteryOnDataRef = XPLMFindDataRef(sPANEL_BATTERY_ON_DR);
	gPanelGeneratorOnDataRef = XPLMFindDataRef(sPANEL_GENERATOR_ON_DR);
	gPanelAvionicsOnDataRef = XPLMFindDataRef(sPANEL_AVIONICS_ON_DR);

	return 1;
}

PLUGIN_API void XPluginDisable(void) {
	XPLMDebugString("-> CP: XPluginDisable\n");
	gRpThreadData.stop = 1;
	gMpThreadData.stop = 1;
	gSpThreadData.stop = 1;
	gMcpThreadData.stop = 1;
	gCbThreadData.stop = 1;
#if IBM
	Sleep(500);
#else
	usleep(50);
#endif
}

PLUGIN_API void XPluginStop(void) {
	XPLMUnregisterFlightLoopCallback(PanelFlightLoopCallback, NULL);
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


