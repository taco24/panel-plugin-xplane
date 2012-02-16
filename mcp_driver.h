#ifndef MCP_DRIVER_H_
#define MCP_DRIVER_H_

#define MCP_OUT_BUF_SIZE              (6)
#define MCP_IN_BUF_SIZE               (8)

#define MCP_READ_KNOB_COURSE_L_MASK  (0x000000F0)
#define MCP_READ_KNOB_HEADING_MASK   (0x0000000F)
#define MCP_READ_KNOB_IAS_MACH_MASK  (0x0000F000)
#define MCP_READ_KNOB_VERTSPEED_MASK (0x00000F00)
#define MCP_READ_KNOB_ALTITUE_MASK   (0x00F00000)
#define MCP_READ_KNOB_COURSE_R_MASK  (0x000F0000)
#define MCP_READ_BUTTONS_MASK        (0x00B04010)
#define MCP_READ_PUSH_BUTTONS_MASK   (0xEFBFEF00)
#define MCP_READ_BTN_VNAV            (0x00000100)
#define MCP_READ_BTN_LNAV            (0x00000200)
#define MCP_READ_BTN_A_CMD           (0x00000400)
#define MCP_READ_BTN_B_CMD           (0x00000800)
#define MCP_READ_BTN_A_T_ARM         (0x00001000)
#define MCP_READ_BTN_C_O             (0x00002000)
#define MCP_READ_BTN_SPD_INTV        (0x00004000)
#define MCP_READ_BTN_VOR_LOC         (0x00008000)
#define MCP_READ_BTN_ALT_INTV        (0x00010000)
#define MCP_READ_BTN_CWS_L           (0x00020000)
#define MCP_READ_BTN_CWS_R           (0x00040000)
#define MCP_READ_BTN_IAS_MACH        (0x00080000)
#define MCP_READ_BTN_HEADING         (0x00100000)
#define MCP_READ_BTN_ALTITUDE        (0x00200000)
#define MCP_READ_BTN_F_D             (0x00400000)
#define MCP_READ_BTN_N1              (0x00800000)
#define MCP_READ_BTN_SPEED           (0x01000000)
#define MCP_READ_BTN_LVL_CHANGE      (0x02000000)
#define MCP_READ_BTN_HDG_SEL         (0x04000000)
#define MCP_READ_BTN_APP             (0x08000000)
#define MCP_READ_BTN_ALT_HLD         (0x00001000)
#define MCP_READ_BTN_V_S             (0x00002000)

#define MCP_LED_PLUS_SIGN    (0x0A)
#define MCP_LED_MINUS_SIGN   (0x0E)

extern int mcp_panel_open();
extern int mcp_panel_write(unsigned char *buf);
extern int mcp_panel_read_non_blocking(unsigned char *buf);
extern int mcp_panel_close();

extern unsigned char mcp_course_left[4];
extern unsigned char mcp_course_right[4];
extern unsigned char mcp_ias_mach[6];
extern unsigned char mcp_heading[4];
extern unsigned char mcp_altitude[6];
extern unsigned char mcp_vert_speed[6];

#endif /* MCP_DRIVER_H_ */
