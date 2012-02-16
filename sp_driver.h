#ifndef SP_DRIVER_H_
#define SP_DRIVER_H_

#define SP_OUT_BUF_SIZE              (2)
#define SP_IN_BUF_SIZE               (4)

#define SP_READ_MASTER_BAT_MASK       (0x000001)
#define SP_READ_MASTER_ALT_MASK       (0x000002)
#define SP_READ_AVIONICS_MASTER_MASK  (0x000004)
#define SP_READ_FUEL_PUMP_MASK        (0x000008)
#define SP_READ_DE_ICE_MASK           (0x000010)
#define SP_READ_PITOT_HEAT_MASK       (0x000020)
#define SP_READ_COWL_MASK             (0x000040)
#define SP_READ_LIGHTS_PANEL_MASK     (0x000080)
#define SP_READ_LIGHTS_BEACON_MASK    (0x000100)
#define SP_READ_LIGHTS_NAV_MASK       (0x000200)
#define SP_READ_LIGHTS_STROBE_MASK    (0x000400)
#define SP_READ_LIGHTS_TAXI_MASK      (0x000800)
#define SP_READ_LIGHTS_LANDING_MASK   (0x001000)
#define SP_READ_ENGINES_KNOB_MASK     (0x03E000)
#define SP_READ_GEARLEVER_DOWN_MASK   (0x040000)
#define SP_READ_GEARLEVER_UP_MASK     (0x080000)

extern int sp_panel_open();
extern int sp_panel_write(unsigned char *buf);
extern int sp_panel_read_non_blocking(unsigned char *buf);
extern int sp_panel_close();


#endif /* SP_DRIVER_H_ */
