#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "sp_controller.h"
#include "sp_driver.h"
#include "defs.h"
#include "time.h"
#include "log.h"
#include "utils.h"

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

// Switch panel
enum SP_COMMANDS_MAP {
	SP_CMD_MAGNETOS_OFF,
	SP_CMD_MAGNETOS_RIGHT,
	SP_CMD_MAGNETOS_LEFT,
	SP_CMD_MAGNETOS_BOTH,
	SP_CMD_ENGINE_START,
	SP_CMD_GENERATOR_ON,
	SP_CMD_GENERATOR_OFF,
	SP_CMD_MASTER_BATTERY_ON,
	SP_CMD_MASTER_BATTERY_OFF,
	SP_CMD_MASTER_ALT_BATTERY_ON,
	SP_CMD_MASTER_ALT_BATTERY_OFF,
	SP_CMD_MASTER_AVIONICS_ON,
	SP_CMD_MASTER_AVIONICS_OFF,
	SP_CMD_FUEL_PUMP_ON,
	SP_CMD_FUEL_PUMP_OFF,
	SP_CMD_DEICE_ON,
	SP_CMD_DEICE_OFF,
	SP_CMD_PITOT_HEAT_ON,
	SP_CMD_PITOT_HEAT_OFF,
	SP_CMD_COWL_CLOSED,
	SP_CMD_COWL_OPEN,
	SP_CMD_LIGHTS_PANEL_ON,
	SP_CMD_LIGHTS_PANEL_OFF,
	SP_CMD_LIGHTS_BEACON_ON,
	SP_CMD_LIGHTS_BEACON_OFF,
	SP_CMD_LIGHTS_NAV_ON,
	SP_CMD_LIGHTS_NAV_OFF,
	SP_CMD_LIGHTS_STROBE_ON,
	SP_CMD_LIGHTS_STROBE_OFF,
	SP_CMD_LIGHTS_TAXI_ON,
	SP_CMD_LIGHTS_TAXI_OFF,
	SP_CMD_LIGHTS_LANDING_ON,
	SP_CMD_LIGHTS_LANDING_OFF,
	SP_CMD_LANDING_GEAR_UP,
	SP_CMD_LANDING_GEAR_DOWN
};

static const int min_mainloop_time = 5000;
static long last_mainloop_idle = 0;
static struct sp_thread_data *gPtrThreadData;
static unsigned char buf[SP_IN_BUF_SIZE];
static unsigned char writeBuf[SP_OUT_BUF_SIZE];
static char tmp[100];
static uint32_t gEngineKnob = 0;
static uint32_t gEngineKnobState = 0;

/* SWITCH PANEL Command Refs */
XPLMCommandRef gSpMagnetosOff1CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff2CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff3CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff4CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff5CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff6CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff7CmdRef = NULL;
XPLMCommandRef gSpMagnetosOff8CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight1CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight2CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight3CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight4CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight5CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight6CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight7CmdRef = NULL;
XPLMCommandRef gSpMagnetosRight8CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft1CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft2CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft3CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft4CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft5CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft6CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft7CmdRef = NULL;
XPLMCommandRef gSpMagnetosLeft8CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth1CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth2CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth3CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth4CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth5CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth6CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth7CmdRef = NULL;
XPLMCommandRef gSpMagnetosBoth8CmdRef = NULL;
XPLMCommandRef gSpEngineStart1CmdRef = NULL;
XPLMCommandRef gSpEngineStart2CmdRef = NULL;
XPLMCommandRef gSpEngineStart3CmdRef = NULL;
XPLMCommandRef gSpEngineStart4CmdRef = NULL;
XPLMCommandRef gSpEngineStart5CmdRef = NULL;
XPLMCommandRef gSpEngineStart6CmdRef = NULL;
XPLMCommandRef gSpEngineStart7CmdRef = NULL;
XPLMCommandRef gSpEngineStart8CmdRef = NULL;
XPLMCommandRef gSpMasterBatteryOnCmdRef = NULL;
XPLMCommandRef gSpMasterBatteryOffCmdRef = NULL;
XPLMCommandRef gSpMasterAltBatteryOnCmdRef = NULL;
XPLMCommandRef gSpMasterAltBatteryOffCmdRef = NULL;
XPLMCommandRef gSpGeneratorOn1CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn2CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn3CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn4CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn5CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn6CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn7CmdRef = NULL;
XPLMCommandRef gSpGeneratorOn8CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff1CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff2CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff3CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff4CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff5CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff6CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff7CmdRef = NULL;
XPLMCommandRef gSpGeneratorOff8CmdRef = NULL;
XPLMCommandRef gSpMasterAvionicsOnCmdRef = NULL;
XPLMCommandRef gSpMasterAvionicsOffCmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn1CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn2CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn3CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn4CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn5CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn6CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn7CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOn8CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff1CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff2CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff3CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff4CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff5CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff6CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff7CmdRef = NULL;
XPLMCommandRef gSpFuelPumpOff8CmdRef = NULL;
XPLMCommandRef gSpDeIceOnCmdRef = NULL;
XPLMCommandRef gSpDeIceOffCmdRef = NULL;
XPLMCommandRef gSpPitotHeatOnCmdRef = NULL;
XPLMCommandRef gSpPitotHeatOffCmdRef = NULL;
XPLMCommandRef gSpCowlClosedCmdRef = NULL;
XPLMCommandRef gSpCowlOpenCmdRef = NULL;
XPLMCommandRef gSpLightsPanelOnCmdRef = NULL;
XPLMCommandRef gSpLightsPanelOffCmdRef = NULL;
XPLMCommandRef gSpLightsBeaconOnCmdRef = NULL;
XPLMCommandRef gSpLightsBeaconOffCmdRef = NULL;
XPLMCommandRef gSpLightsNavOnCmdRef = NULL;
XPLMCommandRef gSpLightsNavOffCmdRef = NULL;
XPLMCommandRef gSpLightsStrobeOnCmdRef = NULL;
XPLMCommandRef gSpLightsStrobeOffCmdRef = NULL;
XPLMCommandRef gSpLightsTaxiOnCmdRef = NULL;
XPLMCommandRef gSpLightsTaxiOffCmdRef = NULL;
XPLMCommandRef gSpLightsLandingOnCmdRef = NULL;
XPLMCommandRef gSpLightsLandingOffCmdRef = NULL;
XPLMCommandRef gSpLandingGearUpCmdRef = NULL;
XPLMCommandRef gSpLandingGearDownCmdRef = NULL;

/* SWITCH PANEL Data Refs */
XPLMDataRef gSpNumberOfBatteriesDataRef = NULL;
XPLMDataRef gSpNumberOfGeneratorsDataRef = NULL;
XPLMDataRef gSpNumberOfEnginesDataRef = NULL;
XPLMDataRef gSpBatteryArrayOnDataRef = NULL;
XPLMDataRef gSpCowlFlapsDataRef = NULL;
XPLMDataRef gSpCockpitLightsDataRef = NULL;
XPLMDataRef gSpAntiIceDataRef = NULL;
XPLMDataRef gSpGearRetractDataRef = NULL;
XPLMDataRef gSpOnGroundDataRef = NULL;
XPLMDataRef gSpLandingGearStatusDataRef = NULL;
XPLMDataRef gSpGear1FailDataRef = NULL;
XPLMDataRef gSpGear2FailDataRef = NULL;
XPLMDataRef gSpGear3FailDataRef = NULL;

/* SWITCH PANEL Global variables */
uint32_t gSpNumberOfBatteries = 0;
uint32_t gSpNumberOfGenerators = 0;
uint32_t gSpNumberOfEngines = 0;
uint32_t gSpBatteryArrayOn = 0;
uint32_t gSpGearRetract = 0;
uint32_t gSpOnGround = 0;
static uint32_t gSpOnGroundCounter = 0;
float gSpLandingGearStatus[10];
uint32_t gSpGear1Fail = 0;
uint32_t gSpGear2Fail = 0;
uint32_t gSpGear3Fail = 0;
float gSpOpencowl[8], gSpClosecowl[8];
int gSpBatArrayOn[8];
int gSpGearSwitchUp;
static uint32_t failed1up;
static uint32_t failed2up;
static uint32_t failed3up;
static uint32_t failed1dn;
static uint32_t failed2dn;
static uint32_t failed3dn;
static unsigned char gearled;
static unsigned char gearledPrev;

int SwitchPanelCommandHandler(XPLMCommandRef    inCommand,
                             XPLMCommandPhase  inPhase,
                             void *            inRefcon) {
//	XPLMDebugString("-> CP: SwitchPanelCommandHandler: start.\n");
//	char Buffer[256];
//	sprintf(Buffer,"Cmdh handler: 0x%08x, %d, 0x%08x\n", inCommand, inPhase, inRefcon);
//	XPLMDebugString(Buffer);
	int status = CMD_PASS_EVENT;

 switch ((int)(inRefcon)) {
		case SP_CMD_LANDING_GEAR_UP:
		case SP_CMD_LANDING_GEAR_DOWN:
			gSpGearRetract = (XPLMGetDatai(gSpGearRetractDataRef));
			gSpOnGround = (XPLMGetDatai(gSpOnGroundDataRef));
			XPLMGetDatavf(gSpLandingGearStatusDataRef, gSpLandingGearStatus, 0, 10);
			gSpGear1Fail = (XPLMGetDatai(gSpGear1FailDataRef));
			gSpGear2Fail = (XPLMGetDatai(gSpGear2FailDataRef));
			gSpGear3Fail = (XPLMGetDatai(gSpGear3FailDataRef));
			break;
		default:
			break;
 }

 return status;
}


inline void sp_led_update() {

	if (gSpGearRetract > 0) {
		if(gSpOnGround > 0) {
			gSpOnGroundCounter = 0;
	    	if(gSpGearSwitchUp == 1){
	    		gearled = 0x07;
	    	} else {
	    		gearled = 0x38;
	    	}
	    	return;
	    } else {
			gSpOnGroundCounter++;
	    	if (gSpOnGroundCounter < 10) {
	    		return;
	    	}
	    }

		XPLMGetDatavf(gSpLandingGearStatusDataRef, gSpLandingGearStatus, 0, 10);
        // Gear is down and locked
        if (gSpLandingGearStatus[0] > 0.99) {
            gearled |= (1<<0);   // * set bit 0 in gearled to 1 *
            gearled &= ~(1<<3);   // * clear bit 3 in gearled to 0 *
        }
        if (gSpLandingGearStatus[1] > 0.99) {
            gearled |= (1<<1);   // * set bit 1 in gearled to 1 *
            gearled &= ~(1<<4);   // * clear bit 4 in gearled to 0 *
        }
        if (gSpLandingGearStatus[2] > 0.99) {
            gearled |= (1<<2);   // * set bit 2 in gearled to 1 *
            gearled &= ~(1<<5);   // * clear bit 5 in gearled to 0 *
        }
        // Gear is in motion
        if ((gSpLandingGearStatus[0] > 0.01) &&(gSpLandingGearStatus[0] < 0.99)) {
            gearled |= (1<<3);   // * set bit 3 in gearled to 1 *
            gearled &= ~(1<<0);   // * clear bit 0 in gearled to 0 *
        }
        if ((gSpLandingGearStatus[1] > 0.01) &&(gSpLandingGearStatus[1] < 0.99)) {
            gearled |= (1<<4);   // * set bit 4 in gearled to 1 *
            gearled &= ~(1<<1);   // * clear bit 1 in gearled to 0 *
        }
        if ((gSpLandingGearStatus[2] > 0.01) &&(gSpLandingGearStatus[2] < 0.99)) {
            gearled |= (1<<5);   // * set bit 5 in gearled to 1 *
            gearled &= ~(1<<2);   // * clear bit 2 in gearled to 0 *
        }
        // Gear is up
        if (gSpLandingGearStatus[0] < 0.01) {
            gearled &= ~(1<<0);   // * clear bit 0 in gearled to 0 *
            gearled &= ~(1<<3);   // * clear bit 3 in gearled to 0 *
        }
        if (gSpLandingGearStatus[1] < 0.01) {
            gearled &= ~(1<<1);   // * clear bit 1 in gearled to 0 *
            gearled &= ~(1<<4);   // * clear bit 4 in gearled to 0 *
        }
        if (gSpLandingGearStatus[2] < 0.01) {
            gearled &= ~(1<<2);   // * clear bit 2 in gearled to 0 *
            gearled &= ~(1<<5);   // * clear bit 5 in gearled to 0 *
        }
        if (gSpGearSwitchUp == 1) {
            // Nose gear had failed
        	if (failed1dn == 0) {
            	if (gSpGear1Fail == 6) {
            		failed1up = 1;
            	}
            	if (gSpGear1Fail == 0) {
            		failed1up = 0;
            	}
            	if (failed1up == 1) {
            		gearled &= ~(1<<3);   // * set bit 3 in gearled to 1 *
            		gearled &= ~(1<<0);   // * clear bit 1 in gearled to 0 *
            	}

            }
            if (failed1dn == 1) {
                gearled |= (1<<3);   // * set bit 3 in gearled to 1 *
                gearled &= ~(1<<0);   // * clear bit 0 in gearled to 0 *
            }

            // Left gear has failed
            if (failed2dn == 0) {
            	if (gSpGear2Fail == 6) {
            		failed2up = 1;
            	}
            	if (gSpGear2Fail == 0) {
            		failed2up = 0;
            	}
            	if (failed2up == 1) {
            		gearled &= ~(1<<4);   // * set bit 3 in gearled to 1 *
            		gearled &= ~(1<<1);   // * clear bit 1 in gearled to 0 *
            	}
            }
            if (failed2dn == 1) {
                gearled |= (1<<4);   // * set bit 3 in gearled to 1 *
                gearled &= ~(1<<1);   // * clear bit 0 in gearled to 0 *
            }

            // Right gear has failed
            if (failed3dn == 0) {
            	if (gSpGear3Fail == 6) {
            		failed3up = 1;
            	}
            	if (gSpGear3Fail == 0) {
            		failed3up = 0;
            	}
            	if (failed3up == 1) {
            		gearled &= ~(1<<5);   // * set bit 3 in gearled to 1 *
            		gearled &= ~(1<<2);   // * clear bit 1 in gearled to 0 *
            	}
            }
            if (failed3dn == 1) {
                gearled |= (1<<5);   // * set bit 3 in gearled to 1 *
                gearled &= ~(1<<2);   // * clear bit 0 in gearled to 0 *
            }
        }
        if (gSpGearSwitchUp == 0) {
           // Nose gear has failed
           if (failed1up == 0) {
        	   if (gSpGear1Fail == 6) {
        		   failed1dn = 1;
        	   }
        	   if (gSpGear1Fail == 0) {
        		   failed1dn = 0;
        	   }
        	   if (failed1dn == 1) {
        		   gearled |= (1<<0);   // * set bit 0 in gearled to 1 *
        		   gearled &= ~(1<<3);   // * clear bit 3 in gearled to 0 *
        	   }
           }
           if (failed1up == 1) {
               gearled |= (1<<3);   // * set bit 3 in gearled to 1 *
               gearled &= ~(1<<0);   // * clear bit 0 in gearled to 0 *
           }

           // Left gear has failed
           if (failed2up == 0) {
        	   if (gSpGear2Fail == 6) {
        		   failed2dn = 1;
        	   }
        	   if (gSpGear2Fail == 0) {
        		   failed2dn = 0;
        	   }
        	   if (failed2dn == 1) {
        		   gearled |= (1<<1);   // * set bit 0 in gearled to 1 *
        		   gearled &= ~(1<<4);   // * clear bit 3 in gearled to 0 *
        	   }

           }
           if (failed2up == 1) {
               gearled |= (1<<4);   // * set bit 3 in gearled to 1 *
               gearled &= ~(1<<1);   // * clear bit 0 in gearled to 0 *
           }

           // Right gear has failed
           if (failed3up == 0) {
        	   if (gSpGear3Fail == 6) {
        		   failed3dn = 1;
        	   }
        	   if (gSpGear3Fail == 0) {
        		   failed3dn = 0;
        	   }
        	   if (failed3dn == 1) {
        		   gearled |= (1<<2);   // * set bit 0 in gearled to 1 *
        		   gearled &= ~(1<<5);   // * clear bit 3 in gearled to 0 *
        	   }

           }
           if (failed3up == 1) {
               gearled |= (1<<5);   // * set bit 3 in gearled to 1 *
               gearled &= ~(1<<2);   // * clear bit 0 in gearled to 0 *
           }

        }

	} else {
		gearled = 0x00;
	}
}

void sp_process_knob(uint32_t knobSelection) {
	switch (knobSelection) {
	case 0x002000:
		if (gSpNumberOfEngines == 1) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
		} else if (gSpNumberOfEngines == 2) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
			XPLMCommandOnce(gSpMagnetosOff2CmdRef);
		} else if (gSpNumberOfEngines == 3) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
			XPLMCommandOnce(gSpMagnetosOff2CmdRef);
			XPLMCommandOnce(gSpMagnetosOff3CmdRef);
		} else if (gSpNumberOfEngines == 4) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
			XPLMCommandOnce(gSpMagnetosOff2CmdRef);
			XPLMCommandOnce(gSpMagnetosOff3CmdRef);
			XPLMCommandOnce(gSpMagnetosOff4CmdRef);
		}
		break;
	case 0x004000:
		if (gSpNumberOfEngines == 1) {
			XPLMCommandOnce(gSpMagnetosRight1CmdRef);
		} else if (gSpNumberOfEngines == 2) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
			XPLMCommandOnce(gSpMagnetosRight2CmdRef);
		} else if (gSpNumberOfEngines == 3) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
			XPLMCommandOnce(gSpMagnetosRight2CmdRef);
			XPLMCommandOnce(gSpMagnetosRight3CmdRef);
		} else if (gSpNumberOfEngines == 4) {
			XPLMCommandOnce(gSpMagnetosOff1CmdRef);
			XPLMCommandOnce(gSpMagnetosOff2CmdRef);
			XPLMCommandOnce(gSpMagnetosRight3CmdRef);
			XPLMCommandOnce(gSpMagnetosRight4CmdRef);
		}
		break;
	case 0x008000:
		if (gSpNumberOfEngines == 1) {
			XPLMCommandOnce(gSpMagnetosLeft1CmdRef);
		} else if (gSpNumberOfEngines == 2) {
			XPLMCommandOnce(gSpMagnetosLeft1CmdRef);
			XPLMCommandOnce(gSpMagnetosOff2CmdRef);
		} else if (gSpNumberOfEngines == 3) {
			XPLMCommandOnce(gSpMagnetosLeft1CmdRef);
			XPLMCommandOnce(gSpMagnetosOff2CmdRef);
			XPLMCommandOnce(gSpMagnetosOff3CmdRef);
		} else if (gSpNumberOfEngines == 4) {
			XPLMCommandOnce(gSpMagnetosLeft1CmdRef);
			XPLMCommandOnce(gSpMagnetosLeft2CmdRef);
			XPLMCommandOnce(gSpMagnetosOff3CmdRef);
			XPLMCommandOnce(gSpMagnetosOff4CmdRef);
		}
		break;
	case 0x010000:
		if (gSpNumberOfEngines == 1) {
			XPLMCommandOnce(gSpMagnetosBoth1CmdRef);
		} else if (gSpNumberOfEngines == 2) {
			XPLMCommandOnce(gSpMagnetosBoth1CmdRef);
			XPLMCommandOnce(gSpMagnetosBoth2CmdRef);
		} else if (gSpNumberOfEngines == 3) {
			XPLMCommandOnce(gSpMagnetosBoth1CmdRef);
			XPLMCommandOnce(gSpMagnetosBoth2CmdRef);
			XPLMCommandOnce(gSpMagnetosBoth3CmdRef);
		} else if (gSpNumberOfEngines == 4) {
			XPLMCommandOnce(gSpMagnetosBoth1CmdRef);
			XPLMCommandOnce(gSpMagnetosBoth2CmdRef);
			XPLMCommandOnce(gSpMagnetosBoth3CmdRef);
			XPLMCommandOnce(gSpMagnetosBoth4CmdRef);
		}
		break;
	case 0x020000:
		if (gSpNumberOfEngines == 1) {
			XPLMCommandOnce(gSpEngineStart1CmdRef);
		} else if (gSpNumberOfEngines == 2) {
			XPLMCommandOnce(gSpEngineStart1CmdRef);
			XPLMCommandOnce(gSpEngineStart2CmdRef);
		} else if (gSpNumberOfEngines == 3) {
			XPLMCommandOnce(gSpEngineStart1CmdRef);
			XPLMCommandOnce(gSpEngineStart2CmdRef);
			XPLMCommandOnce(gSpEngineStart3CmdRef);
		} else if (gSpNumberOfEngines == 4) {
			XPLMCommandOnce(gSpEngineStart1CmdRef);
			XPLMCommandOnce(gSpEngineStart2CmdRef);
			XPLMCommandOnce(gSpEngineStart3CmdRef);
			XPLMCommandOnce(gSpEngineStart4CmdRef);
		}
		break;
	default:
		break;
	}
}

int sp_process(uint32_t msg) {
    //sprintf(tmp, "-> CP: sp_controller.sp_process: msg: %d\n", msg);
	//XPLMDebugString(tmp);
	int res = 0;
    gEngineKnob = msg & SP_READ_ENGINES_KNOB_MASK;
    uint32_t landingGearUp = msg & SP_READ_GEARLEVER_UP_MASK;
    uint32_t landingGearDown = msg & SP_READ_GEARLEVER_DOWN_MASK;
    uint32_t lightsLanding = msg & SP_READ_LIGHTS_LANDING_MASK;
    uint32_t lightsTaxi = msg & SP_READ_LIGHTS_TAXI_MASK;
    uint32_t lightsStrobe = msg & SP_READ_LIGHTS_STROBE_MASK;
    uint32_t lightsNav = msg & SP_READ_LIGHTS_NAV_MASK;
    uint32_t lightsBeacon = msg & SP_READ_LIGHTS_BEACON_MASK;
    uint32_t lightsPanel = msg & SP_READ_LIGHTS_PANEL_MASK;
    uint32_t cowl = msg & SP_READ_COWL_MASK;
    uint32_t pitot = msg & SP_READ_PITOT_HEAT_MASK;
    uint32_t deice = msg & SP_READ_DE_ICE_MASK;
    uint32_t fuelPump = msg & SP_READ_FUEL_PUMP_MASK;
    uint32_t avionicsMaster = msg & SP_READ_AVIONICS_MASTER_MASK;
    uint32_t masterBattery = msg & SP_READ_MASTER_BAT_MASK;
    uint32_t masterAltBattery = msg & SP_READ_MASTER_ALT_MASK;

    if (gEngineKnob) {
    	gEngineKnobState = gEngineKnob;
    	sp_process_knob(gEngineKnob);
    }
    if (landingGearUp) {
    	gSpGearSwitchUp = 1;
    	XPLMCommandOnce(gSpLandingGearUpCmdRef);
    } else if (landingGearDown) {
    	gSpGearSwitchUp = 0;
    	XPLMCommandOnce(gSpLandingGearDownCmdRef);
    }

    if (lightsLanding) {
    	XPLMCommandOnce(gSpLightsLandingOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpLightsLandingOffCmdRef);
    }
    if (lightsTaxi) {
    	XPLMCommandOnce(gSpLightsTaxiOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpLightsTaxiOffCmdRef);
    }
    if (lightsPanel) {
    	XPLMSetDataf(gSpCockpitLightsDataRef, 1);
    } else {
    	XPLMSetDataf(gSpCockpitLightsDataRef, 0);
    }
    if (lightsBeacon) {
    	XPLMCommandOnce(gSpLightsBeaconOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpLightsBeaconOffCmdRef);
    }
    if (lightsNav) {
    	XPLMCommandOnce(gSpLightsNavOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpLightsNavOffCmdRef);
    }
    if (lightsStrobe) {
    	XPLMCommandOnce(gSpLightsStrobeOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpLightsStrobeOffCmdRef);
    }
    if (masterBattery) {
    	if (gSpNumberOfBatteries == 1) {
    		gSpBatArrayOn[0] = 1;
    	} else if (gSpNumberOfBatteries == 2) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    	} else if (gSpNumberOfBatteries == 3) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    		gSpBatArrayOn[2] = 1;
    	} else if (gSpNumberOfBatteries == 4) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    		gSpBatArrayOn[2] = 1;
    		gSpBatArrayOn[3] = 1;
    	} else if (gSpNumberOfBatteries == 5) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    		gSpBatArrayOn[2] = 1;
    		gSpBatArrayOn[3] = 1;
    		gSpBatArrayOn[4] = 1;
    	} else if (gSpNumberOfBatteries == 6) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    		gSpBatArrayOn[2] = 1;
    		gSpBatArrayOn[3] = 1;
    		gSpBatArrayOn[4] = 1;
    		gSpBatArrayOn[5] = 1;
    	} else if (gSpNumberOfBatteries == 7) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    		gSpBatArrayOn[2] = 1;
    		gSpBatArrayOn[3] = 1;
    		gSpBatArrayOn[4] = 1;
    		gSpBatArrayOn[5] = 1;
    		gSpBatArrayOn[6] = 1;
    	} else if (gSpNumberOfBatteries == 8) {
    		gSpBatArrayOn[0] = 1;
    		gSpBatArrayOn[1] = 1;
    		gSpBatArrayOn[2] = 1;
    		gSpBatArrayOn[3] = 1;
    		gSpBatArrayOn[4] = 1;
    		gSpBatArrayOn[5] = 1;
    		gSpBatArrayOn[6] = 1;
    		gSpBatArrayOn[7] = 1;
    	}
    	XPLMSetDatavi(gSpBatteryArrayOnDataRef, gSpBatArrayOn, 0, 8);
    } else {
    	if (gSpNumberOfBatteries == 1) {
    		gSpBatArrayOn[0] = 0;
    	} else if (gSpNumberOfBatteries == 2) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    	} else if (gSpNumberOfBatteries == 3) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    		gSpBatArrayOn[2] = 0;
    	} else if (gSpNumberOfBatteries == 4) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    		gSpBatArrayOn[2] = 0;
    		gSpBatArrayOn[3] = 0;
    	} else if (gSpNumberOfBatteries == 5) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    		gSpBatArrayOn[2] = 0;
    		gSpBatArrayOn[3] = 0;
    		gSpBatArrayOn[4] = 0;
    	} else if (gSpNumberOfBatteries == 6) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    		gSpBatArrayOn[2] = 0;
    		gSpBatArrayOn[3] = 0;
    		gSpBatArrayOn[4] = 0;
    		gSpBatArrayOn[5] = 0;
    	} else if (gSpNumberOfBatteries == 7) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    		gSpBatArrayOn[2] = 0;
    		gSpBatArrayOn[3] = 0;
    		gSpBatArrayOn[4] = 0;
    		gSpBatArrayOn[5] = 0;
    		gSpBatArrayOn[6] = 0;
    	} else if (gSpNumberOfBatteries == 8) {
    		gSpBatArrayOn[0] = 0;
    		gSpBatArrayOn[1] = 0;
    		gSpBatArrayOn[2] = 0;
    		gSpBatArrayOn[3] = 0;
    		gSpBatArrayOn[4] = 0;
    		gSpBatArrayOn[5] = 0;
    		gSpBatArrayOn[6] = 0;
    		gSpBatArrayOn[7] = 0;
    	}
    	XPLMSetDatavi(gSpBatteryArrayOnDataRef, gSpBatArrayOn, 0, 8);
    }
    if (masterAltBattery) {
    	XPLMCommandOnce(gSpMasterAltBatteryOnCmdRef);
    	if (gSpNumberOfGenerators == 1) {
        	XPLMCommandOnce(gSpGeneratorOn1CmdRef);
    	} else if (gSpNumberOfGenerators == 2) {
        	XPLMCommandOnce(gSpGeneratorOn1CmdRef);
        	XPLMCommandOnce(gSpGeneratorOn2CmdRef);
    	} else if (gSpNumberOfGenerators == 3) {
        	XPLMCommandOnce(gSpGeneratorOn1CmdRef);
        	XPLMCommandOnce(gSpGeneratorOn2CmdRef);
        	XPLMCommandOnce(gSpGeneratorOn3CmdRef);
    	} else if (gSpNumberOfGenerators == 4) {
        	XPLMCommandOnce(gSpGeneratorOn1CmdRef);
        	XPLMCommandOnce(gSpGeneratorOn2CmdRef);
        	XPLMCommandOnce(gSpGeneratorOn3CmdRef);
        	XPLMCommandOnce(gSpGeneratorOn4CmdRef);
    	}
    } else {
    	XPLMCommandOnce(gSpMasterAltBatteryOffCmdRef);
    	if (gSpNumberOfGenerators == 1) {
        	XPLMCommandOnce(gSpGeneratorOff1CmdRef);
    	} else if (gSpNumberOfGenerators == 2) {
        	XPLMCommandOnce(gSpGeneratorOff1CmdRef);
        	XPLMCommandOnce(gSpGeneratorOff2CmdRef);
    	} else if (gSpNumberOfGenerators == 3) {
        	XPLMCommandOnce(gSpGeneratorOff1CmdRef);
        	XPLMCommandOnce(gSpGeneratorOff2CmdRef);
        	XPLMCommandOnce(gSpGeneratorOff3CmdRef);
    	} else if (gSpNumberOfGenerators == 4) {
        	XPLMCommandOnce(gSpGeneratorOff1CmdRef);
        	XPLMCommandOnce(gSpGeneratorOff2CmdRef);
        	XPLMCommandOnce(gSpGeneratorOff3CmdRef);
        	XPLMCommandOnce(gSpGeneratorOff4CmdRef);
    	}
    }
    if (avionicsMaster) {
    	XPLMCommandOnce(gSpMasterAvionicsOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpMasterAvionicsOffCmdRef);
    }
    if (fuelPump) {
    	if (gSpNumberOfEngines == 1) {
        	XPLMCommandOnce(gSpFuelPumpOn1CmdRef);
    	} else if (gSpNumberOfEngines == 2) {
        	XPLMCommandOnce(gSpFuelPumpOn1CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOn2CmdRef);
    	} else if (gSpNumberOfEngines == 3) {
        	XPLMCommandOnce(gSpFuelPumpOn1CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOn2CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOn3CmdRef);
    	} else if (gSpNumberOfEngines == 4) {
        	XPLMCommandOnce(gSpFuelPumpOn1CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOn2CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOn3CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOn4CmdRef);
    	}
    } else {
    	if (gSpNumberOfEngines == 1) {
        	XPLMCommandOnce(gSpFuelPumpOff1CmdRef);
    	} else if (gSpNumberOfEngines == 2) {
        	XPLMCommandOnce(gSpFuelPumpOff1CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOff2CmdRef);
    	} else if (gSpNumberOfEngines == 3) {
        	XPLMCommandOnce(gSpFuelPumpOff1CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOff2CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOff3CmdRef);
    	} else if (gSpNumberOfEngines == 4) {
        	XPLMCommandOnce(gSpFuelPumpOff1CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOff2CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOff3CmdRef);
        	XPLMCommandOnce(gSpFuelPumpOff4CmdRef);
    	}
    }
    if (deice) {
    	XPLMSetDatai(gSpAntiIceDataRef, 1);
    } else {
    	XPLMSetDatai(gSpAntiIceDataRef, 0);
    }
    if (pitot) {
    	XPLMCommandOnce(gSpPitotHeatOnCmdRef);
    } else {
    	XPLMCommandOnce(gSpPitotHeatOffCmdRef);
    }
    if (cowl) {
    	if (gSpNumberOfEngines == 1) {
    		gSpOpencowl[0] = 1;
    	} else if (gSpNumberOfEngines == 2) {
    		gSpOpencowl[0] = 1;
    		gSpOpencowl[1] = 1;
    	} else if (gSpNumberOfEngines == 3) {
    		gSpOpencowl[0] = 1;
    		gSpOpencowl[1] = 1;
    		gSpOpencowl[2] = 1;
    	} else if (gSpNumberOfEngines == 4) {
    		gSpOpencowl[0] = 1;
    		gSpOpencowl[1] = 1;
    		gSpOpencowl[2] = 1;
    		gSpOpencowl[3] = 1;
    	}
    	XPLMSetDatavf(gSpCowlFlapsDataRef, gSpOpencowl, 0, 8);
    } else {
    	if (gSpNumberOfEngines == 1) {
    		gSpClosecowl[0] = 1;
    	} else if (gSpNumberOfEngines == 2) {
    		gSpClosecowl[0] = 1;
    		gSpClosecowl[1] = 1;
    	} else if (gSpNumberOfEngines == 3) {
    		gSpClosecowl[0] = 1;
    		gSpClosecowl[1] = 1;
    		gSpClosecowl[2] = 1;
    	} else if (gSpNumberOfEngines == 4) {
    		gSpClosecowl[0] = 1;
    		gSpClosecowl[1] = 1;
    		gSpClosecowl[2] = 1;
    		gSpClosecowl[3] = 1;
    	}
    	XPLMSetDatavf(gSpCowlFlapsDataRef, gSpClosecowl, 0, 8);
    }

    return res;
}

void sp_update_datarefs() {
    gSpNumberOfBatteries = (XPLMGetDatai(gSpNumberOfBatteriesDataRef));
    gSpNumberOfGenerators = (XPLMGetDatai(gSpNumberOfGeneratorsDataRef));
    gSpNumberOfEngines = (XPLMGetDatai(gSpNumberOfEnginesDataRef));
    gSpBatteryArrayOn = (XPLMGetDatai(gSpBatteryArrayOnDataRef));
    gSpGearRetract = (XPLMGetDatai(gSpGearRetractDataRef));
    gSpOnGround = (XPLMGetDatai(gSpOnGroundDataRef));
    XPLMGetDatavf(gSpLandingGearStatusDataRef, gSpLandingGearStatus, 0, 10);
    gSpGear1Fail = (XPLMGetDatai(gSpGear1FailDataRef));
    gSpGear2Fail = (XPLMGetDatai(gSpGear2FailDataRef));
    gSpGear3Fail = (XPLMGetDatai(gSpGear3FailDataRef));
}

void sp_init() {
	XPLMDebugString("-> CP: sp_controller.sp_init.\n");
    gSpMagnetosOff1CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_1_CR);
    gSpMagnetosOff2CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_2_CR);
    gSpMagnetosOff3CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_3_CR);
    gSpMagnetosOff4CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_4_CR);
    gSpMagnetosOff5CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_5_CR);
    gSpMagnetosOff6CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_6_CR);
    gSpMagnetosOff7CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_7_CR);
    gSpMagnetosOff8CmdRef  = XPLMFindCommand(sSP_MAGNETOS_OFF_8_CR);
    gSpMagnetosRight1CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_1_CR);
    gSpMagnetosRight2CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_2_CR);
    gSpMagnetosRight3CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_3_CR);
    gSpMagnetosRight4CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_4_CR);
    gSpMagnetosRight5CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_5_CR);
    gSpMagnetosRight6CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_6_CR);
    gSpMagnetosRight7CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_7_CR);
    gSpMagnetosRight8CmdRef  = XPLMFindCommand(sSP_MAGNETOS_RIGHT_8_CR);
    gSpMagnetosLeft1CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_1_CR);
    gSpMagnetosLeft2CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_2_CR);
    gSpMagnetosLeft3CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_3_CR);
    gSpMagnetosLeft4CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_4_CR);
    gSpMagnetosLeft5CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_5_CR);
    gSpMagnetosLeft6CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_6_CR);
    gSpMagnetosLeft7CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_7_CR);
    gSpMagnetosLeft8CmdRef  = XPLMFindCommand(sSP_MAGNETOS_LEFT_8_CR);
    gSpMagnetosBoth1CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_1_CR);
    gSpMagnetosBoth2CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_2_CR);
    gSpMagnetosBoth3CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_3_CR);
    gSpMagnetosBoth4CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_4_CR);
    gSpMagnetosBoth5CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_5_CR);
    gSpMagnetosBoth6CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_6_CR);
    gSpMagnetosBoth7CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_7_CR);
    gSpMagnetosBoth8CmdRef  = XPLMFindCommand(sSP_MAGNETOS_BOTH_8_CR);
    gSpEngineStart1CmdRef  = XPLMFindCommand(sSP_ENGINE_START_1_CR);
    gSpEngineStart2CmdRef  = XPLMFindCommand(sSP_ENGINE_START_2_CR);
    gSpEngineStart3CmdRef  = XPLMFindCommand(sSP_ENGINE_START_3_CR);
    gSpEngineStart4CmdRef  = XPLMFindCommand(sSP_ENGINE_START_4_CR);
    gSpEngineStart5CmdRef  = XPLMFindCommand(sSP_ENGINE_START_5_CR);
    gSpEngineStart6CmdRef  = XPLMFindCommand(sSP_ENGINE_START_6_CR);
    gSpEngineStart7CmdRef  = XPLMFindCommand(sSP_ENGINE_START_7_CR);
    gSpEngineStart8CmdRef  = XPLMFindCommand(sSP_ENGINE_START_8_CR);

    // battery
    gSpMasterBatteryOnCmdRef  = XPLMFindCommand(sSP_MASTER_BATTERY_ON_CR);
    gSpMasterBatteryOffCmdRef  = XPLMFindCommand(sSP_MASTER_BATTERY_OFF_CR);
    gSpMasterAltBatteryOnCmdRef  = XPLMFindCommand(sSP_MASTER_ALT_BATTERY_ON_CR);
    gSpMasterAltBatteryOffCmdRef  = XPLMFindCommand(sSP_MASTER_ALT_BATTERY_OFF_CR);

    gSpGeneratorOn1CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_1_CR);
    gSpGeneratorOff1CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_1_CR);
    gSpGeneratorOn2CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_2_CR);
    gSpGeneratorOff2CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_2_CR);
    gSpGeneratorOn3CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_3_CR);
    gSpGeneratorOff3CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_3_CR);
    gSpGeneratorOn4CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_4_CR);
    gSpGeneratorOff4CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_4_CR);
    gSpGeneratorOn5CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_5_CR);
    gSpGeneratorOff5CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_5_CR);
    gSpGeneratorOn6CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_6_CR);
    gSpGeneratorOff6CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_6_CR);
    gSpGeneratorOn7CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_7_CR);
    gSpGeneratorOff7CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_7_CR);
    gSpGeneratorOn8CmdRef = XPLMFindCommand(sSP_GENERATOR_ON_8_CR);
    gSpGeneratorOff8CmdRef = XPLMFindCommand(sSP_GENERATOR_OFF_8_CR);

    // various
    gSpMasterAvionicsOnCmdRef  = XPLMFindCommand(sSP_MASTER_AVIONICS_ON_CR);
    gSpMasterAvionicsOffCmdRef  = XPLMFindCommand(sSP_MASTER_AVIONICS_OFF_CR);
    gSpFuelPumpOn1CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_1_CR);
    gSpFuelPumpOff1CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_1_CR);
    gSpFuelPumpOn2CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_2_CR);
    gSpFuelPumpOff2CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_2_CR);
    gSpFuelPumpOn3CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_3_CR);
    gSpFuelPumpOff3CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_3_CR);
    gSpFuelPumpOn4CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_4_CR);
    gSpFuelPumpOff4CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_4_CR);
    gSpFuelPumpOn5CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_5_CR);
    gSpFuelPumpOff5CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_5_CR);
    gSpFuelPumpOn6CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_6_CR);
    gSpFuelPumpOff6CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_6_CR);
    gSpFuelPumpOn7CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_7_CR);
    gSpFuelPumpOff7CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_7_CR);
    gSpFuelPumpOn8CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_ON_8_CR);
    gSpFuelPumpOff8CmdRef  = XPLMFindCommand(sSP_FUEL_PUMP_OFF_8_CR);
    gSpDeIceOnCmdRef  = XPLMFindCommand(sSP_DE_ICE_ON_CR);
    gSpDeIceOffCmdRef  = XPLMFindCommand(sSP_DE_ICE_OFF_CR);
    gSpPitotHeatOnCmdRef  = XPLMFindCommand(sSP_PITOT_HEAT_ON_CR);
    gSpPitotHeatOffCmdRef  = XPLMFindCommand(sSP_PITOT_HEAT_OFF_CR);

    gSpCowlClosedCmdRef  = XPLMFindCommand(sSP_COWL_CLOSED_CR);
    gSpCowlOpenCmdRef  = XPLMFindCommand(sSP_COWL_OPEN_CR);

    // lights
    gSpLightsPanelOnCmdRef  = XPLMFindCommand(sSP_LIGHTS_PANEL_ON_CR);
    gSpLightsPanelOffCmdRef  = XPLMFindCommand(sSP_LIGHTS_PANEL_OFF_CR);
    gSpLightsBeaconOnCmdRef  = XPLMFindCommand(sSP_LIGHTS_BEACON_ON_CR);
    gSpLightsBeaconOffCmdRef  = XPLMFindCommand(sSP_LIGHTS_BEACON_OFF_CR);
    gSpLightsNavOnCmdRef  = XPLMFindCommand(sSP_LIGHTS_NAV_ON_CR);
    gSpLightsNavOffCmdRef  = XPLMFindCommand(sSP_LIGHTS_NAV_OFF_CR);
    gSpLightsStrobeOnCmdRef  = XPLMFindCommand(sSP_LIGHTS_STROBE_ON_CR);
    gSpLightsStrobeOffCmdRef  = XPLMFindCommand(sSP_LIGHTS_STROBE_OFF_CR);
    gSpLightsTaxiOnCmdRef  = XPLMFindCommand(sSP_LIGHTS_TAXI_ON_CR);
    gSpLightsTaxiOffCmdRef  = XPLMFindCommand(sSP_LIGHTS_TAXI_OFF_CR);
    gSpLightsLandingOnCmdRef  = XPLMFindCommand(sSP_LIGHTS_LANDING_ON_CR);
    gSpLightsLandingOffCmdRef  = XPLMFindCommand(sSP_LIGHTS_LANDING_OFF_CR);

    // landing gear
    gSpLandingGearUpCmdRef  = XPLMFindCommand(sSP_LANDING_GEAR_UP_CR);
    gSpLandingGearDownCmdRef  = XPLMFindCommand(sSP_LANDING_GEAR_DOWN_CR);

    XPLMRegisterCommandHandler(gSpMagnetosOff1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MAGNETOS_OFF);
    XPLMRegisterCommandHandler(gSpMagnetosRight1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MAGNETOS_RIGHT);
    XPLMRegisterCommandHandler(gSpMagnetosLeft1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MAGNETOS_LEFT);
    XPLMRegisterCommandHandler(gSpMagnetosBoth1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MAGNETOS_BOTH);
    XPLMRegisterCommandHandler(gSpEngineStart1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_ENGINE_START);
    XPLMRegisterCommandHandler(gSpMasterBatteryOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MASTER_BATTERY_ON);
    XPLMRegisterCommandHandler(gSpMasterBatteryOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MASTER_BATTERY_OFF);
    XPLMRegisterCommandHandler(gSpMasterAltBatteryOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MASTER_ALT_BATTERY_ON);
    XPLMRegisterCommandHandler(gSpMasterAltBatteryOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MASTER_ALT_BATTERY_OFF);
    XPLMRegisterCommandHandler(gSpGeneratorOn1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_GENERATOR_ON);
    XPLMRegisterCommandHandler(gSpGeneratorOff1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_GENERATOR_OFF);
    XPLMRegisterCommandHandler(gSpMasterAvionicsOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MASTER_AVIONICS_ON);
    XPLMRegisterCommandHandler(gSpMasterAvionicsOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_MASTER_AVIONICS_OFF);
    XPLMRegisterCommandHandler(gSpFuelPumpOn1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_FUEL_PUMP_ON);
    XPLMRegisterCommandHandler(gSpFuelPumpOff1CmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_FUEL_PUMP_OFF);
    XPLMRegisterCommandHandler(gSpDeIceOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_DEICE_ON);
    XPLMRegisterCommandHandler(gSpDeIceOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_DEICE_OFF);
    XPLMRegisterCommandHandler(gSpPitotHeatOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_PITOT_HEAT_ON);
    XPLMRegisterCommandHandler(gSpPitotHeatOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_PITOT_HEAT_OFF);
    XPLMRegisterCommandHandler(gSpCowlClosedCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_COWL_CLOSED);
    XPLMRegisterCommandHandler(gSpCowlOpenCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_COWL_OPEN);

    XPLMRegisterCommandHandler(gSpLightsPanelOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_PANEL_ON);
    XPLMRegisterCommandHandler(gSpLightsPanelOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_PANEL_OFF);
    XPLMRegisterCommandHandler(gSpLightsBeaconOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_BEACON_ON);
    XPLMRegisterCommandHandler(gSpLightsBeaconOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_BEACON_OFF);
    XPLMRegisterCommandHandler(gSpLightsNavOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_NAV_ON);
    XPLMRegisterCommandHandler(gSpLightsNavOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_NAV_OFF);
    XPLMRegisterCommandHandler(gSpLightsStrobeOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_STROBE_ON);
    XPLMRegisterCommandHandler(gSpLightsStrobeOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_STROBE_OFF);
    XPLMRegisterCommandHandler(gSpLightsTaxiOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_TAXI_ON);
    XPLMRegisterCommandHandler(gSpLightsTaxiOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_TAXI_OFF);
    XPLMRegisterCommandHandler(gSpLightsLandingOnCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_LANDING_ON);
    XPLMRegisterCommandHandler(gSpLightsLandingOffCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LIGHTS_LANDING_OFF);
    XPLMRegisterCommandHandler(gSpLandingGearUpCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LANDING_GEAR_UP);
    XPLMRegisterCommandHandler(gSpLandingGearDownCmdRef, SwitchPanelCommandHandler, CMD_HNDLR_PROLOG, (void *) SP_CMD_LANDING_GEAR_DOWN);

    /*----- SwitchPanel Data Ref Assignment -----*/
    gSpNumberOfBatteriesDataRef = XPLMFindDataRef(sSP_NUMBER_OF_BATTERIES_DR);
    gSpNumberOfGeneratorsDataRef = XPLMFindDataRef(sSP_NUMBER_OF_GENERATORS_DR);
    gSpNumberOfEnginesDataRef = XPLMFindDataRef(sSP_NUMBER_OF_ENGINES_DR);
    gSpGearRetractDataRef = XPLMFindDataRef(sSP_GEAR_RETRACT_DR);
    gSpOnGroundDataRef = XPLMFindDataRef(sSP_ON_GROUND_DR);
    gSpLandingGearStatusDataRef = XPLMFindDataRef(sSP_LANDING_GEAR_STATUS_DR);
    gSpGear1FailDataRef = XPLMFindDataRef(sSP_GEAR_1_FAIL_DR);
    gSpGear2FailDataRef = XPLMFindDataRef(sSP_GEAR_2_FAIL_DR);
    gSpGear3FailDataRef = XPLMFindDataRef(sSP_GEAR_3_FAIL_DR);
    gSpAntiIceDataRef = XPLMFindDataRef(sSP_ANTI_ICE_DR);
    gSpCowlFlapsDataRef = XPLMFindDataRef(sSP_COWL_FLAPS_DR);
    gSpCockpitLightsDataRef = XPLMFindDataRef(sSP_COCKPIT_LIGHTS_DR);
    gSpBatteryArrayOnDataRef = XPLMFindDataRef(sSP_BATTERY_ARRAY_ON_DR);
    sp_update_datarefs();
}

void *spRun(void *ptr_thread_data) {
	int inReportBytesCount = 0;

#if IBM
		Sleep(SLEEP_TIME * 3);
#else
		usleep(SLEEP_TIME * 3);
#endif

	sp_init();

	gPtrThreadData = (struct sp_thread_data *) ptr_thread_data;

	int result = sp_panel_open();
	if (result < 0) {
		XPLMDebugString("-> CP: sp_controller.spRun: shutdown thread.\n");
		pthread_exit(NULL);
		return 0;
	}
	XPLMDebugString("-> CP: sp_controller.spRun: panel opened.\n");

	last_mainloop_idle = sys_time_clock_get_time_usec();
	// while stop == 0 calculate position.
	while (gPtrThreadData->stop == 0) {
		long loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// Read panel for new messages. CRITICAL FAST 100 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(10000, COUNTER3, loop_start_time)) {
			// read/write board
			sp_led_update();
			inReportBytesCount = sp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: sp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				}
				uint32_t msg = 0;
				msg += buf[2] << 16;
				msg += buf[1] << 8;
				msg += buf[0];
				sp_process(msg);
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// Update Panel. CRITICAL 20 Hz functions:
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(50000, COUNTER4, loop_start_time)) {
			// Update local DataRefs.
			sp_update_datarefs();
			// update Panel.
			inReportBytesCount = sp_panel_read_non_blocking(buf);
			if (inReportBytesCount > 0) {
				if (inReportBytesCount > 3) {
					sprintf(tmp, "-> CP: sp_controller.run: bytes in report %d: %#0x,%#0x,%#0x\n", inReportBytesCount, buf[2], buf[1], buf[0]);
					XPLMDebugString(tmp);
				}
				uint32_t msg = 0;
				msg += buf[2] << 16;
				msg += buf[1] << 8;
				msg += buf[0];
				sp_process(msg);
			}
		    if (gEngineKnobState == 0x020000) {
		    	// Start engine
		    	if (gSpNumberOfEngines == 1) {
					XPLMCommandOnce(gSpEngineStart1CmdRef);
		    	} else if (gSpNumberOfEngines == 2) {
					XPLMCommandOnce(gSpEngineStart1CmdRef);
					XPLMCommandOnce(gSpEngineStart2CmdRef);
		    	} else if (gSpNumberOfEngines == 3) {
					XPLMCommandOnce(gSpEngineStart1CmdRef);
					XPLMCommandOnce(gSpEngineStart2CmdRef);
					XPLMCommandOnce(gSpEngineStart3CmdRef);
		    	} else if (gSpNumberOfEngines == 4) {
					XPLMCommandOnce(gSpEngineStart1CmdRef);
					XPLMCommandOnce(gSpEngineStart2CmdRef);
					XPLMCommandOnce(gSpEngineStart3CmdRef);
					XPLMCommandOnce(gSpEngineStart4CmdRef);
		    	}
		    }
		    if (gSpGearRetract > 0) {
		    	if (gearledPrev != gearled) {
		    		gearledPrev = gearled;
			    	writeBuf[1] = gearled;
					sp_panel_write(writeBuf);
		    	}
		    }
		}
		///////////////////////////////////////////////////////////////////////////

		if (loop_start_time - last_mainloop_idle >= MAX_DELAY_TIME) {
			XPLMDebugString("-> CP: sp_controller.run: CRITICAL WARNING! CPU LOAD TOO HIGH.\n");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		} else {
			//writeConsole(0, 0, "CPU LOAD OK.");
		}

		// wait 1 milliseconds
#if IBM
		Sleep(SLEEP_TIME * 3);
#else
		usleep(SLEEP_TIME * 3);
#endif
	}
	sp_panel_close();
#if IBM
		Sleep(SLEEP_TIME * 3);
#else
		usleep(SLEEP_TIME * 3);
#endif
	pthread_exit(NULL);
	return 0;
}

