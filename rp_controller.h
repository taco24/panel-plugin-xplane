#ifndef RP_CONTROLLER_H_
#define RP_CONTROLLER_H_

#include <pthread.h>

extern void *rpRun(void *ptr_thread_data);

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

#endif /* RP_CONTROLLER_H_ */
