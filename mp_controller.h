#ifndef MP_CONTROLLER_H_
#define MP_CONTROLLER_H_

#include <pthread.h>
#include "defs.h"

extern void *mpRun(void *ptr_thread_data);
extern void mp_process_trimwheel();


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

#define sMP_PITCH_TRIM_DOWN_CR              "sim/flight_controls/pitch_trim_down"
#define sMP_PITCH_TRIM_UP_CR                "sim/flight_controls/pitch_trim_up"
#define sMP_PITCH_TRIM_TAKEOFF_CR           "sim/flight_controls/pitch_trim_takeoff"
#define sMP_FLAPS_DOWN_CR                   "sim/flight_controls/flaps_down"
#define sMP_FLAPS_UP_CR                     "sim/flight_controls/flaps_up"

#define sMP_SERVOS_AND_FLIGHT_DIR_ON_CR     "sim/autopilot/servos_and_flight_dir_on"
#define sMP_SERVOS_AND_FLIGHT_DIR_OFF_CR    "sim/autopilot/servos_and_flight_dir_off"
#define sMP_AP_HEADING_CR                   "sim/autopilot/heading"
#define sMP_AP_NAV_CR                       "sim/autopilot/NAV"
#define sMP_AP_LEVEL_CHANGE_CR              "sim/autopilot/level_change"
#define sMP_AP_ALTITUDE_HOLD_CR             "sim/autopilot/altitude_hold"
#define sMP_AP_VERTICAL_SPEED_CR            "sim/autopilot/vertical_speed"
#define sMP_AP_APPROACH_CR                  "sim/autopilot/approach"
#define sMP_AP_BACK_COURSE_CR               "sim/autopilot/back_course"

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


#define sMP_AP_FLIGHT_DIR_MODE_DR           "sim/cockpit2/autopilot/flight_director_mode"
#define sMP_AP_HEADING_STATUS_DR            "sim/cockpit2/autopilot/heading_status"
#define sMP_AP_NAV_STATUS_DR                "sim/cockpit2/autopilot/nav_status"
#define sMP_AP_SPEED_STATUS_DR              "sim/cockpit2/autopilot/speed_status"
#define sMP_AP_ALTITUDE_HOLD_STATUS_DR      "sim/cockpit2/autopilot/altitude_hold_status"
#define sMP_AP_VVI_STATUS_DR                "sim/cockpit2/autopilot/vvi_status"
#define sMP_AP_APPROACH_STATUS_DR           "sim/cockpit2/autopilot/approach_status"
#define sMP_AP_BACKCOURSE_STATUS_DR         "sim/cockpit2/autopilot/backcourse_status"
#define sMP_AUTOPILOT_STATE_DR              "sim/cockpit/autopilot/autopilot_state"
#define sMP_AUTOPILOT_MODE_DR               "sim/cockpit/autopilot/autopilot_mode"

#endif /* MP_CONTROLLER_H_ */
