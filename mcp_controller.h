#ifndef MCP_CONTROLLER_H_
#define MCP_CONTROLLER_H_

#include <pthread.h>
#include "defs.h"

extern void *mcpRun(void *ptr_thread_data);


// Multi Panel
#define sMCP_ALTITUDE_DOWN_CR                "sim/autopilot/altitude_down"
#define sMCP_ALTITUDE_UP_CR                  "sim/autopilot/altitude_up"
#define sMCP_VERTICAL_SPEED_DOWN_CR          "sim/autopilot/vertical_speed_down"
#define sMCP_VERTICAL_SPEED_UP_CR            "sim/autopilot/vertical_speed_up"
#define sMCP_AIRSPEED_DOWN_CR                "sim/autopilot/airspeed_down"
#define sMCP_AIRSPEED_UP_CR                  "sim/autopilot/airspeed_up"
#define sMCP_HEADING_DOWN_CR                 "sim/autopilot/heading_down"
#define sMCP_HEADING_UP_CR                   "sim/autopilot/heading_up"
#define sMCP_OBS_HSI_DOWN_CR                 "sim/radios/obs_HSI_down"
#define sMCP_OBS_HSI_UP_CR                   "sim/radios/obs_HSI_up"

#define sMCP_PITCH_TRIM_DOWN_CR              "sim/flight_controls/pitch_trim_down"
#define sMCP_PITCH_TRIM_UP_CR                "sim/flight_controls/pitch_trim_up"
#define sMCP_PITCH_TRIM_TAKEOFF_CR           "sim/flight_controls/pitch_trim_takeoff"
#define sMCP_FLAPS_DOWN_CR                   "sim/flight_controls/flaps_down"
#define sMCP_FLAPS_UP_CR                     "sim/flight_controls/flaps_up"

#define sMCP_FDIR_SERVOS_UP_ONE_CR           "sim/autopilot/fdir_servos_up_one"
#define sMCP_FDIR_SERVOS_DOWN_ONE_CR         "sim/autopilot/fdir_servos_down_one"
#define sMCP_SERVOS_AND_FLIGHT_DIR_OFF_CR    "sim/autopilot/servos_and_flight_dir_off"
#define sMCP_SERVOS_AND_FLIGHT_DIR_ON_CR     "sim/autopilot/servos_and_flight_dir_on"
#define sMCP_AP_HEADING_SYNC_CR              "sim/autopilot/heading_sync"
#define sMCP_AP_HEADING_CR                   "sim/autopilot/heading"
#define sMCP_AP_NAV_CR                       "sim/autopilot/NAV"
#define sMCP_AP_VNAV_CR                      "sim/autopilot/vnav"
#define sMCP_AP_LEVEL_CHANGE_CR              "sim/autopilot/level_change"
#define sMCP_AP_ALTITUDE_HOLD_CR             "sim/autopilot/altitude_hold"
#define sMCP_AP_ALTITUDE_SYNC_CR             "sim/autopilot/altitude_sync"
#define sMCP_AP_AIRSPEED_SYNC_CR             "sim/autopilot/airspeed_sync"
#define sMCP_AP_VERTICAL_SPEED_CR            "sim/autopilot/vertical_speed"
#define sMCP_AP_APPROACH_CR                  "sim/autopilot/approach"
#define sMCP_AP_BACK_COURSE_CR               "sim/autopilot/back_course"
#define sMCP_AP_AUTOTHROTTLE_ON_CR           "sim/autopilot/autothrottle_on"
#define sMCP_AP_AUTOTHROTTLE_OFF_CR          "sim/autopilot/autothrottle_off"
#define sMCP_AP_GLIDESLOPE_CR                "sim/autopilot/glide_slope"
#define sMCP_AP_TOGA_CR                      "sim/autopilot/take_off_go_around "

// data refs
#define sMCP_AUTOPILOT_MODE_DR               "sim/cockpit/autopilot/autopilot_mode"
#define sMCP_AUTOPILOT_STATE_DR              "sim/cockpit/autopilot/autopilot_state"
#define sMCP_ALTITUDE_DIAL_FT_DR             "sim/cockpit2/autopilot/altitude_dial_ft"
#define sMCP_ALTITUDE_HOLD_FT_DR             "sim/cockpit2/autopilot/altitude_hold_ft"
#define sMCP_ALTITUDE_VNAV_FT_DR             "sim/cockpit2/autopilot/altitude_vnav_ft"
#define sMCP_VVI_DIAL_FPM_DR                 "sim/cockpit2/autopilot/vvi_dial_fpm"
#define sMCP_AIRSPEED_DR                     "sim/cockpit2/autopilot/airspeed_dial_kts_mach"
//#define sMCP_HEADING_DIAL_DEG_MAG_PILOT_DR   "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"

#define sMCP_HSI_BEARING_DEG_MAG_PILOT_DR    "sim/cockpit/autopilot/heading_mag"
#define sMCP_HSI_OBS_DEG_MAG_PILOT_DR        "sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot"
#define sMCP_HSI_SOURCE_SELECT_PILOT_DR      "sim/cockpit2/radios/actuators/HSI_source_select_pilot"
#define sMCP_NAV1_COURSE_DEG_MAG_PILOT_DR    "sim/cockpit2/radios/actuators/nav1_course_deg_mag_pilot"
#define sMCP_NAV2_COURSE_DEG_MAG_PILOT_DR    "sim/cockpit2/radios/actuators/nav2_course_deg_mag_pilot"
#define sMCP_GPS_COURSE_DR                   "sim/cockpit/gps/course"


#define sMCP_AP_HEADING_STATUS_DR            "sim/cockpit2/autopilot/heading_status"
#define sMCP_AP_NAV_STATUS_DR                "sim/cockpit2/autopilot/nav_status"
#define sMCP_AP_SPEED_STATUS_DR              "sim/cockpit2/autopilot/speed_status"
#define sMCP_AP_ALTITUDE_HOLD_STATUS_DR      "sim/cockpit2/autopilot/altitude_hold_status"
#define sMCP_AP_VVI_STATUS_DR                "sim/cockpit2/autopilot/vvi_status"
#define sMCP_AP_APPROACH_STATUS_DR           "sim/cockpit2/autopilot/approach_status"
#define sMCP_AP_BACKCOURSE_STATUS_DR         "sim/cockpit2/autopilot/backcourse_status"
#define sMCP_AP_GLIDESLOPE_STATUS_DR         "sim/cockpit2/autopilot/glideslope_status"
#define sMCP_AP_VNAV_STATUS_DR               "sim/cockpit2/autopilot/vnav_status"
#define sMCP_AP_VNAV_ARMED_DR                "sim/cockpit2/autopilot/vnav_armed"
#define sMCP_AP_HNAV_STATUS_DR               "sim/cockpit2/autopilot/hnav_armed"
#define sMCP_AP_GLIDE_SLOPE_STATUS_DR        "sim/cockpit2/autopilot/glideslope_status"
#define sMCP_AP_TOGA_STATUS_DR               "sim/cockpit2/autopilot/TOGA_status"
#define sMCP_AP_TOGA_LATERAL_STATUS_DR       "sim/cockpit2/autopilot/TOGA_lateral_status"
#define sMCP_AP_PITCH_STATUS_DR              "sim/cockpit2/autopilot/pitch_status"

#define sMCP_AP_AUTOPILOT_ON_DR              "sim/cockpit2/autopilot/autopilot_on"
#define sMCP_AP_AUTOPILOT_SOURCE_DR          "sim/cockpit2/autopilot/autopilot_source"
#define sMCP_AP_FLIGHT_DIRECTOR_MODE_DR      "sim/cockpit2/autopilot/flight_director_mode"
#define sMCP_AP_BACKCOURSE_ON_DR             "sim/cockpit2/autopilot/backcourse_on"
#define sMCP_AP_AIRSPEED_IS_MACH_DR          "sim/cockpit2/autopilot/airspeed_is_mach"
#define sMCP_AP_AUTOTHROTTLE_ENABLED_DR      "sim/cockpit2/autopilot/autothrottle_enabled"
#define sMCP_AP_AUTOTHROTTLE_ON_DR           "sim/cockpit2/autopilot/autothrottle_on"
#define sMCP_AP_HEADING_MODE_DR              "sim/cockpit2/autopilot/heading_mode"
#define sMCP_AP_ALTITUDE_MODE_DR             "sim/cockpit2/autopilot/altitude_mode"
#define sMCP_AP_FLIGHT_DIR_MODE_DR           "sim/cockpit2/autopilot/flight_director_mode"
#define sMCP_AP_AUTOPILOT_ON_DR              "sim/cockpit2/autopilot/autopilot_on"
#define sMCP_AP_ALT_VVI_IS_SHHOWING_VVI_DR   "sim/cockpit2/autopilot/alt_vvi_is_showing_vvi"
#define sMCP_AP_VNAV_ARMED_DR                "sim/cockpit2/autopilot/vnav_armed"
#define sMCP_AP_ALTITUDE_HOLD_ARMED_DR       "sim/cockpit2/autopilot/altitude_hold_armed"
#define sMCP_AP_HNAV_ARMED_DR                "sim/cockpit2/autopilot/hnav_armed"
#define sMCP_AP_GLIDE_SLOPE_DR               "sim/cockpit2/autopilot/glideslope_armed"



#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

#endif /* MCP_CONTROLLER_H_ */
