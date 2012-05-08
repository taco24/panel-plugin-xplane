#ifndef RP_CONTROLLER_H_
#define RP_CONTROLLER_H_

#include <pthread.h>

extern void *rpRun(void *ptr_thread_data);
extern void rp_process_power(uint32_t gPanelBatteryOn, uint32_t gPanelGeneratorOn, uint32_t gPanelAvionicsOn);

// Radio Panel
#define sRP_STDBY_COM1_FINE_DOWN_CR       "sim/radios/stby_com1_fine_down"
#define sRP_STDBY_COM1_FINE_UP_CR         "sim/radios/stby_com1_fine_up"
#define sRP_STDBY_COM1_COARSE_DOWN_CR     "sim/radios/stby_com1_coarse_down"
#define sRP_STDBY_COM1_COARSE_UP_CR       "sim/radios/stby_com1_coarse_up"

#define sRP_ACTV_COM1_FINE_DOWN_CR       "sim/radios/actv_com1_fine_down"
#define sRP_ACTV_COM1_FINE_UP_CR         "sim/radios/actv_com1_fine_up"
#define sRP_ACTV_COM1_COARSE_DOWN_CR     "sim/radios/actv_com1_coarse_down"
#define sRP_ACTV_COM1_COARSE_UP_CR       "sim/radios/actv_com1_coarse_up"

#define sRP_COM1_STBY_FLIP_CR             "sim/radios/com1_standy_flip"

#define sRP_COM1_FREQ_HZ_DR               "sim/cockpit/radios/com1_freq_hz"
#define sRP_COM1_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/com1_stdby_freq_hz"


#define sRP_STDBY_COM2_FINE_DOWN_CR       "sim/radios/stby_com2_fine_down"
#define sRP_STDBY_COM2_FINE_UP_CR         "sim/radios/stby_com2_fine_up"
#define sRP_STDBY_COM2_COARSE_DOWN_CR     "sim/radios/stby_com2_coarse_down"
#define sRP_STDBY_COM2_COARSE_UP_CR       "sim/radios/stby_com2_coarse_up"

#define sRP_ACTV_COM2_FINE_DOWN_CR       "sim/radios/actv_com2_fine_down"
#define sRP_ACTV_COM2_FINE_UP_CR         "sim/radios/actv_com2_fine_up"
#define sRP_ACTV_COM2_COARSE_DOWN_CR     "sim/radios/actv_com2_coarse_down"
#define sRP_ACTV_COM2_COARSE_UP_CR       "sim/radios/actv_com2_coarse_up"

#define sRP_COM2_STBY_FLIP_CR             "sim/radios/com2_standy_flip"

#define sRP_COM2_FREQ_HZ_DR               "sim/cockpit/radios/com2_freq_hz"
#define sRP_COM2_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/com2_stdby_freq_hz"

#define sRP_STDBY_NAV1_FINE_DOWN_CR       "sim/radios/stby_nav1_fine_down"
#define sRP_STDBY_NAV1_FINE_UP_CR         "sim/radios/stby_nav1_fine_up"
#define sRP_STDBY_NAV1_COARSE_DOWN_CR     "sim/radios/stby_nav1_coarse_down"
#define sRP_STDBY_NAV1_COARSE_UP_CR       "sim/radios/stby_nav1_coarse_up"

#define sRP_ACTV_NAV1_FINE_DOWN_CR       "sim/radios/actv_nav1_fine_down"
#define sRP_ACTV_NAV1_FINE_UP_CR         "sim/radios/actv_nav1_fine_up"
#define sRP_ACTV_NAV1_COARSE_DOWN_CR     "sim/radios/actv_nav1_coarse_down"
#define sRP_ACTV_NAV1_COARSE_UP_CR       "sim/radios/actv_nav1_coarse_up"

#define sRP_NAV1_STBY_FLIP_CR             "sim/radios/nav1_standy_flip"

#define sRP_NAV1_FREQ_HZ_DR               "sim/cockpit/radios/nav1_freq_hz"
#define sRP_NAV1_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/nav1_stdby_freq_hz"


#define sRP_STDBY_NAV2_FINE_DOWN_CR       "sim/radios/stby_nav2_fine_down"
#define sRP_STDBY_NAV2_FINE_UP_CR         "sim/radios/stby_nav2_fine_up"
#define sRP_STDBY_NAV2_COARSE_DOWN_CR     "sim/radios/stby_nav2_coarse_down"
#define sRP_STDBY_NAV2_COARSE_UP_CR       "sim/radios/stby_nav2_coarse_up"

#define sRP_ACTV_NAV2_FINE_DOWN_CR       "sim/radios/actv_nav2_fine_down"
#define sRP_ACTV_NAV2_FINE_UP_CR         "sim/radios/actv_nav2_fine_up"
#define sRP_ACTV_NAV2_COARSE_DOWN_CR     "sim/radios/actv_nav2_coarse_down"
#define sRP_ACTV_NAV2_COARSE_UP_CR       "sim/radios/actv_nav2_coarse_up"

#define sRP_NAV2_STBY_FLIP_CR             "sim/radios/nav2_standy_flip"

#define sRP_NAV2_FREQ_HZ_DR               "sim/cockpit/radios/nav2_freq_hz"
#define sRP_NAV2_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/nav2_stdby_freq_hz"


#define sRP_ACTV_ADF1_HUNDREDS_UP_CR      "sim/radios/stby_adf1_hundreds_up"
#define sRP_ACTV_ADF1_HUNDREDS_DOWN_CR    "sim/radios/stby_adf1_hundreds_down"
#define sRP_ACTV_ADF1_TENS_UP_CR          "sim/radios/stby_adf1_tens_up"
#define sRP_ACTV_ADF1_TENS_DOWN_CR        "sim/radios/stby_adf1_tens_down"
#define sRP_ACTV_ADF1_ONES_UP_CR          "sim/radios/stby_adf1_ones_up"
#define sRP_ACTV_ADF1_ONES_DOWN_CR        "sim/radios/stby_adf1_ones_down"

#define sRP_ADF1_STBY_FLIP_CR             "sim/radios/adf1_standy_flip"

#define sRP_ACTV_ADF2_HUNDREDS_UP_CR      "sim/radios/stby_adf2_hundreds_up"
#define sRP_ACTV_ADF2_HUNDREDS_DOWN_CR    "sim/radios/stby_adf2_hundreds_down"
#define sRP_ACTV_ADF2_TENS_UP_CR          "sim/radios/stby_adf2_tens_up"
#define sRP_ACTV_ADF2_TENS_DOWN_CR        "sim/radios/stby_adf2_tens_down"
#define sRP_ACTV_ADF2_ONES_UP_CR          "sim/radios/stby_adf2_ones_up"
#define sRP_ACTV_ADF2_ONES_DOWN_CR        "sim/radios/stby_adf2_ones_down"

#define sRP_ADF2_STBY_FLIP_CR             "sim/radios/adf2_standy_flip"

#define sRP_XPDR_THOUSANDS_UP_CR          "sim/transponder/transponder_thousands_up"
#define sRP_XPDR_THOUSANDS_DOWN_CR        "sim/transponder/transponder_thousands_down"
#define sRP_XPDR_HUNDREDS_UP_CR           "sim/transponder/transponder_hundreds_up"
#define sRP_XPDR_HUNDREDS_DOWN_CR         "sim/transponder/transponder_hundreds_down"
#define sRP_XPDR_TENS_UP_CR               "sim/transponder/transponder_tens_up"
#define sRP_XPDR_TENS_DOWN_CR             "sim/transponder/transponder_tens_down"
#define sRP_XPDR_ONES_UP_CR               "sim/transponder/transponder_ones_up"
#define sRP_XPDR_ONES_DOWN_CR             "sim/transponder/transponder_ones_down"

#define sRP_XPDR_CODE_DR                  "sim/cockpit/radios/transponder_code"
#define sRP_QNH_CODE_DR                   "sim/cockpit/misc/barometer_setting"
#define sRP_ADF1_FREQ_HZ_DR               "sim/cockpit/radios/adf1_freq_hz"
#define sRP_ADF1_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/adf1_stdby_freq_hz"
#define sRP_ADF2_FREQ_HZ_DR               "sim/cockpit/radios/adf2_freq_hz"
#define sRP_ADF2_STDBY_FREQ_HZ_DR         "sim/cockpit/radios/adf2_stdby_freq_hz"
#define sRP_NAV1_DME_DISTANCE_NM_DR       "sim/cockpit2/radios/indicators/nav1_dme_distance_nm"
#define sRP_NAV1_DME_SPEED_KTS_DR         "sim/cockpit2/radios/indicators/nav1_dme_speed_kts"
#define sRP_NAV2_DME_DISTANCE_NM_DR       "sim/cockpit2/radios/indicators/nav2_dme_distance_nm"
#define sRP_NAV2_DME_SPEED_KTS_DR         "sim/cockpit2/radios/indicators/nav2_dme_speed_kts"
#define sRP_DME_SLAVE_SOURCE_DR           "sim/cockpit2/radios/actuators/DME_slave_source"

#endif /* RP_CONTROLLER_H_ */
