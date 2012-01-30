#ifndef MP_CONTROLLER_H_
#define MP_CONTROLLER_H_

#include <pthread.h>
#include "defs.h"

extern void *mpRun(void *ptr_thread_data);


// Multi Panel
#define sMP_ALTITUDE_DOWN_CR                "sim/autopilot/altitude_down"
#define sMP_ALTITUDE_UP_CR                  "sim/autopilot/altitude_up"
#define sMP_VERTICAL_SPEED_DOWN_CR          "sim/autopilot/vertical_speed_down"
#define sMP_VERTICAL_SPEED_UP_CR            "sim/autopilot/vertical_speed_up"
#define sMP_AIRSPEED_DOWN_CR                "sim/autopilot/airspeed_down"
#define sMP_AIRSPEED_UP_CR                  "sim/autopilot/airspeed_up"
#define sMP_HEADING_DOWN_CR                 "sim/autopilot/heading_down"
#define sMP_HEADING_UP_CR                   "sim/autopilot/heading_up"
#define sMP_OBS_HSI_DOWN_CR                 "sim/radios/obs_HSI_down"
#define sMP_OBS_HSI_UP_CR                   "sim/radios/obs_HSI_up"

// data refs
#define sMP_ALTITUDE_DIAL_FT_DR             "sim/cockpit2/autopilot/altitude_dial_ft"
#define sMP_VVI_DIAL_FPM_DR                 "sim/cockpit2/autopilot/vvi_dial_fpm"
#define sMP_AIRSPEED_DR                     "sim/cockpit/autopilot/airspeed"
#define sMP_HEADING_DIAL_DEG_MAG_PILOT_DR   "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"

//#define sMP_HSI_BEARING_DEG_MAG_PILOT        "sim/cockpit2/radios/indicators/hsi_bearing_deg_mag_pilot"
#define sMP_HSI_OBS_DEG_MAG_PILOT_DR        "sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot"
#define sMP_HSI_SOURCE_SELECT_PILOT_DR      "sim/cockpit2/radios/actuators/HSI_source_select_pilot"
#define sMP_NAV1_COURSE_DEG_MAG_PILOT_DR    "sim/cockpit2/radios/actuators/nav1_course_deg_mag_pilot"
#define sMP_NAV2_COURSE_DEG_MAG_PILOT_DR    "sim/cockpit2/radios/actuators/nav2_course_deg_mag_pilot"
#define sMP_GPS_COURSE_DR                   "sim/cockpit/gps/course"


#endif /* MP_CONTROLLER_H_ */
