#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include "sp_driver.h"
#include "hidapi.h"
#include "XPLMUtilities.h"

#define MAX_STR 255

enum {
	HID_ERROR = -1,
	VENDOR_ID = 0x06A3,
	SP_PROD_ID = 0x0D67,
	SP_ERROR_THRESH = 40,
	PANEL_CHECK_INTERVAL = 5
// seconds
};

hid_device *spHandle;
static char tmp[100];
unsigned char sp_in_buf[SP_IN_BUF_SIZE];
unsigned char sp_zero_panel[SP_OUT_BUF_SIZE]   = {0x00, 0x00};
unsigned char sp_blank_panel[SP_OUT_BUF_SIZE]  = {0x00, 0x00};
unsigned char sp_green_panel[SP_OUT_BUF_SIZE]  = {0x00, 0x07};
unsigned char sp_red_panel[SP_OUT_BUF_SIZE]    = {0x00, 0x38};
unsigned char sp_orange_panel[SP_OUT_BUF_SIZE] = {0x00, 0x3F};

int sp_panel_open() {
	int res = 0;

	spHandle = hid_open(VENDOR_ID, SP_PROD_ID, NULL);

	if (!spHandle) {
		XPLMDebugString("-> CP: sp_driver.panel_open: unable to open device.\n");
		return -1;
	}
	wchar_t wstr[MAX_STR];
	res = hid_get_manufacturer_string(spHandle, wstr, MAX_STR);
    sprintf(tmp, "-> CP: sp_driver.panel_open: Manufacturer String %ls\n", wstr);
	XPLMDebugString(tmp);

	hid_set_nonblocking(spHandle, 1);
	res = hid_read(spHandle, sp_in_buf, SP_IN_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: sp_driver.panel_open: Error: %ls\n", hid_error(spHandle));
		XPLMDebugString(tmp);
	}
	res = hid_send_feature_report(spHandle, sp_zero_panel, SP_OUT_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: sp_driver.panel_open: Error: %ls\n", hid_error(spHandle));
		XPLMDebugString(tmp);
	}
	return 0;
}

int sp_panel_write(unsigned char buf[]) {
	int res = 0;
	if (spHandle) {
		res = hid_send_feature_report(spHandle, buf,
				SP_OUT_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: sp_driver.panel_write: Error: %ls\n", hid_error(spHandle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int sp_panel_read_non_blocking(unsigned char *buf) {
	int res = 0;
	if (spHandle) {
		hid_set_nonblocking(spHandle, 1);
		res = hid_read(spHandle, buf, SP_IN_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: sp_driver.panel_read_non_blocking: Error: %ls\n", hid_error(spHandle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int sp_panel_close() {
	int res = 0;
	if (spHandle) {
		res = hid_send_feature_report(spHandle, sp_blank_panel, SP_OUT_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: sp_driver.panel_close: Error: %ls\n", hid_error(spHandle));
			XPLMDebugString(tmp);
		}
		hid_close(spHandle);
		XPLMDebugString("-> CP: sp_driver.panel_close: panel closed.\n");
		spHandle = NULL;
	}
	return res;
}

