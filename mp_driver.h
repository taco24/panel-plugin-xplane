#ifndef MP_DRIVER_H_
#define MP_DRIVER_H_

#define MP_OUT_BUF_SIZE              (13)
#define MP_IN_BUF_SIZE               (4)

#define MP_APBTN_BITPOS             (0)
#define MP_HDGBTN_BITPOS            (1)
#define MP_NAVBTN_BITPOS            (2)
#define MP_IASBTN_BITPOS            (3)
#define MP_ALTBTN_BITPOS            (4)
#define MP_VSBTN_BITPOS             (5)
#define MP_APRBTN_BITPOS            (6)
#define MP_REVBTN_BITPOS            (7)
#define MP_READ_KNOB_MODE_MASK      (0x0000001F)
#define MP_READ_BTNS_MASK           (0x00007F80)
#define MP_READ_FLAPS_MASK          (0x00030000)
#define MP_READ_TRIM_MASK           (0x000C0000)
#define MP_READ_TUNING_MASK         (0x00000060)
#define MP_READ_THROTTLE_MASK       (0x00008000)
#define MP_READ_KNOB_ALT            (0x00000001)
#define MP_READ_KNOB_VS             (0x00000002)
#define MP_READ_KNOB_IAS            (0x00000004)
#define MP_READ_KNOB_HDG            (0x00000008)
#define MP_READ_KNOB_CRS            (0x00000010)
#define MP_READ_TUNING_RIGHT        (0x00000020)
#define MP_READ_TUNING_LEFT         (0x00000040)
#define MP_READ_AP_BTN              (0x00000080)
#define MP_READ_HDG_BTN             (0x00000100)
#define MP_READ_NAV_BTN             (0x00000200)
#define MP_READ_IAS_BTN             (0x00000400)
#define MP_READ_ALT_BTN             (0x00000800)
#define MP_READ_VS_BTN              (0x00001000)
#define MP_READ_APR_BTN             (0x00002000)
#define MP_READ_REV_BTN             (0x00004000)
#define MP_READ_THROTTLE_ON         (0x00008000)
#define MP_READ_THROTTLE_OFF        (0x00000000)
#define MP_READ_FLAPS_UP            (0x00010000)
#define MP_READ_FLAPS_DN            (0x00020000)
#define MP_READ_TRIM_DOWN           (0x00040000)
#define MP_READ_TRIM_UP             (0x00080000)
#define MP_READ_NOMSG               (0xFFFFFFFF)


extern int mp_panel_open();
extern int mp_panel_write(unsigned char *buf);
extern int mp_panel_read_non_blocking(unsigned char *buf);
extern int mp_panel_close();


#endif /* MP_DRIVER_H_ */
