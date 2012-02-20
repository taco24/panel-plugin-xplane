#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include "mcp_driver.h"
#include "hidapi.h"
#include "XPLMUtilities.h"

#define MAX_STR 255

enum {
	HID_ERROR = -1,
	VENDOR_ID = 0x09F3,
	MCP_PROD_ID = 0x0064,
	MCP_ERROR_THRESH = 40,
	PANEL_CHECK_INTERVAL = 5
// seconds
};

hid_device *mcpHandle;
static char tmp[100];
unsigned char mcp_in_buf[MCP_IN_BUF_SIZE];
unsigned char mcp_blank_panel[5] = {0x0F, 0x00, 0x00, 0x00, 0x00};
unsigned char mcp_course_left[4] = {0x03, 0x3F, 0x3F, 0x3F};
unsigned char mcp_course_right[4] = {0x0D, 0x3F, 0x3F, 0x3F};
unsigned char mcp_ias_mach[6] = {0x05, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F};
unsigned char mcp_heading[4] = {0x07, 0x3F, 0x3F, 0x3F};
unsigned char mcp_altitude[6] = {0x09, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F};
unsigned char mcp_vert_speed[6] = {0x0B, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F};
unsigned char mcp_leds[5] = {0x0F, 0x00, 0x00, 0x00, 0x00};
const unsigned char mcp_all_on_panel[5] = {0x0F, 0x00, 0x10, 0x00, 0x00};

int mcp_panel_open() {
	int res = 0;

	mcpHandle = hid_open(VENDOR_ID, MCP_PROD_ID, NULL);

	if (!mcpHandle) {
		XPLMDebugString("-> CP: mcp_driver.panel_open: unable to open device.\n");
		return -1;
	}
	wchar_t wstr[MAX_STR];
	res = hid_get_manufacturer_string(mcpHandle, wstr, MAX_STR);
    sprintf(tmp, "-> CP: mcp_driver.panel_open: Manufacturer String %ls\n", wstr);
	XPLMDebugString(tmp);

	hid_set_nonblocking(mcpHandle, 1);
	res = hid_read(mcpHandle, mcp_in_buf, MCP_IN_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: mcp_driver.panel_open: Error: %ls\n", hid_error(mcpHandle));
		XPLMDebugString(tmp);
	}
	res = hid_send_feature_report(mcpHandle, mcp_all_on_panel, sizeof(mcp_all_on_panel));
	res = hid_send_feature_report(mcpHandle, mcp_course_left, sizeof(mcp_course_left));
	res = hid_send_feature_report(mcpHandle, mcp_course_right, sizeof(mcp_course_right));
	res = hid_send_feature_report(mcpHandle, mcp_ias_mach, sizeof(mcp_ias_mach));
	res = hid_send_feature_report(mcpHandle, mcp_heading, sizeof(mcp_heading));
	res = hid_send_feature_report(mcpHandle, mcp_altitude, sizeof(mcp_altitude));
	res = hid_send_feature_report(mcpHandle, mcp_vert_speed, sizeof(mcp_vert_speed));
	if (res < 0) {
	    sprintf(tmp, "-> CP: mcp_driver.panel_open: Error: %ls\n", hid_error(mcpHandle));
		XPLMDebugString(tmp);
	}
	return 0;
}

int mcp_panel_write(unsigned char buf[]) {
	int res = 0;
	int i = 0;
	unsigned char tmpBuffer[MCP_OUT_BUF_SIZE];
	for (i = 0; i < MCP_OUT_BUF_SIZE; i++) {
		tmpBuffer[i] = buf[i];
	}
	if (mcpHandle) {
		res = hid_send_feature_report(mcpHandle, tmpBuffer,
				sizeof(tmpBuffer));
		if (res < 0) {
		    sprintf(tmp, "-> CP: mcp_driver.panel_write: Error: %ls\n", hid_error(mcpHandle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int mcp_panel_read_non_blocking(unsigned char *buf) {
	int res = 0;
	if (mcpHandle) {
		hid_set_nonblocking(mcpHandle, 1);
		res = hid_read(mcpHandle, buf, MCP_IN_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: mcp_driver.panel_read_non_blocking: Error: %ls\n", hid_error(mcpHandle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int mcp_panel_close() {
	int res = 0;
	if (mcpHandle) {
		res = hid_send_feature_report(mcpHandle, mcp_blank_panel, sizeof(mcp_blank_panel));
		if (res < 0) {
		    sprintf(tmp, "-> CP: mcp_driver.panel_close: Error: %ls\n", hid_error(mcpHandle));
			XPLMDebugString(tmp);
		}
		hid_close(mcpHandle);
		XPLMDebugString("-> CP: mcp_driver.panel_close: panel closed.\n");
		mcpHandle = NULL;
	}
	return res;
}

