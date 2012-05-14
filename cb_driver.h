#ifndef CB_DRIVER_H_
#define CB_DRIVER_H_

#define CB_OUT_BUF_SIZE              (23)
#define CB_IN_BUF_SIZE               (4)

#define CB_READ_UPPER_KNOB_MODE_MASK (0x00007F)
#define CB_READ_UPPER_KNOB_COM1      (0x000001)
#define CB_READ_UPPER_KNOB_COM2      (0x000002)
#define CB_READ_UPPER_KNOB_NAV1      (0x000004)
#define CB_READ_UPPER_KNOB_NAV2      (0x000008)
#define CB_READ_UPPER_KNOB_ADF       (0x000010)
#define CB_READ_UPPER_KNOB_DME       (0x000020)
#define CB_READ_UPPER_KNOB_TRANSPNDR (0x000040)
#define CB_READ_LOWER_KNOB_MODE_MASK (0x003F80)
#define CB_READ_LOWER_KNOB_COM1      (0x000080)
#define CB_READ_LOWER_KNOB_COM2      (0x000100)
#define CB_READ_LOWER_KNOB_NAV1      (0x000200)
#define CB_READ_LOWER_KNOB_NAV2      (0x000400)
#define CB_READ_LOWER_KNOB_ADF       (0x000800)
#define CB_READ_LOWER_KNOB_DME       (0x001000)
#define CB_READ_LOWER_KNOB_TRANSPNDR (0x002000)
#define CB_READ_UPPER_ACT_STBY       (0x004000)
#define CB_READ_LOWER_ACT_STBY       (0x008000)
#define CB_READ_UPPER_FINE_TUNING_MASK (0x030000)
#define CB_READ_UPPER_COARSE_TUNING_MASK (0x0C0000)
#define CB_READ_UPPER_FINE_RIGHT     (0x010000)
#define CB_READ_UPPER_FINE_LEFT      (0x020000)
#define CB_READ_UPPER_COARSE_RIGHT   (0x040000)
#define CB_READ_UPPER_COARSE_LEFT    (0x080000)
#define CB_READ_LOWER_FINE_TUNING_MASK (0x300000)
#define CB_READ_LOWER_COARSE_TUNING_MASK (0xC00000)
#define CB_READ_LOWER_FINE_RIGHT     (0x100000)
#define CB_READ_LOWER_FINE_LEFT      (0x200000)
#define CB_READ_LOWER_COARSE_RIGHT   (0x400000)
#define CB_READ_LOWER_COARSE_LEFT    (0x800000)

extern int cb_panel_open();
extern int cb_panel_write(unsigned char *buf);
extern int cb_panel_write_empty();
extern int cb_panel_read_blocking(unsigned char *buf);
extern int cb_panel_read_non_blocking(unsigned char *buf);
extern int cb_panel_close();

extern unsigned char cb_blank_panel[CB_OUT_BUF_SIZE];
extern unsigned char cb_zero_panel[CB_OUT_BUF_SIZE];


#endif /* CB_DRIVER_H_ */
