#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include "rp_driver.h"
#include "hidapi.h"
#include "XPLMUtilities.h"

#define MAX_STR 255

enum {
	HID_ERROR = -1,
	VENDOR_ID = 0x06A3,
	RP_PROD_ID = 0x0D05,
	RP_ERROR_THRESH = 40,
	PANEL_CHECK_INTERVAL = 5
// seconds
};

hid_device *handle;
char tmp[100];
unsigned char rp_in_buf[RP_IN_BUF_SIZE];
static unsigned char rp_zero_panel[RP_OUT_BUF_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xFF, 0xFF};
static unsigned char rp_blank_panel[RP_OUT_BUF_SIZE] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

int panel_open() {
	int res = 0;

	handle = hid_open(VENDOR_ID, RP_PROD_ID, NULL);

	if (!handle) {
		XPLMDebugString("-> CP: rp_driver.panel_open: unable to open device.\n");
		return 1;
	}
	wchar_t wstr[MAX_STR];
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
    sprintf(tmp, "-> CP: rp_driver.panel_open: Manufacturer String %ls\n", wstr);
	XPLMDebugString(tmp);

	hid_set_nonblocking(handle, 1);
	res = hid_read(handle, rp_in_buf, RP_IN_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: rp_driver.panel_open: Error: %ls\n", hid_error(handle));
		XPLMDebugString(tmp);
	}
	res = hid_send_feature_report(handle, rp_zero_panel, RP_OUT_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: rp_driver.panel_open: Error: %ls\n", hid_error(handle));
		XPLMDebugString(tmp);
	}
	return 0;
}

int panel_write(unsigned char buf[]) {
	int res = 0;
	if (handle) {
		res = hid_send_feature_report(handle, buf,
				RP_OUT_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: rp_driver.panel_write: Error: %ls\n", hid_error(handle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int panel_read_non_blocking(unsigned char *buf) {
	int res = 0;
	if (handle) {
		hid_set_nonblocking(handle, 1);
		res = hid_read(handle, buf, RP_IN_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: rp_driver.panel_read_non_blocking: Error: %ls\n", hid_error(handle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int panel_close() {
	int res = 0;
	if (handle) {
		res = hid_send_feature_report(handle, rp_blank_panel, RP_OUT_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: rp_driver.panel_close: Error: %ls\n", hid_error(handle));
			XPLMDebugString(tmp);
		}
		hid_close(handle);
		XPLMDebugString("-> CP: rp_driver.panel_close: panel closed.\n");
		handle = NULL;
	}
	return res;
}

