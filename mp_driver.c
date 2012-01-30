#if IBM
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include "mp_driver.h"
#include "hidapi.h"
#include "XPLMUtilities.h"

#define MAX_STR 255

enum {
	HID_ERROR = -1,
	VENDOR_ID = 0x06A3,
	MP_PROD_ID = 0x0D06,
	MP_ERROR_THRESH = 40,
	PANEL_CHECK_INTERVAL = 5
// seconds
};

hid_device *mpHandle;
static char tmp[100];
unsigned char mp_in_buf[MP_IN_BUF_SIZE];
unsigned char mp_blank_panel[13] = {0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
                                        0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x00, 0x00};

const unsigned char mp_zero_panel[13] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int mp_panel_open() {
	int res = 0;

	mpHandle = hid_open(VENDOR_ID, MP_PROD_ID, NULL);

	if (!mpHandle) {
		XPLMDebugString("-> CP: mp_driver.panel_open: unable to open device.\n");
		return 1;
	}
	wchar_t wstr[MAX_STR];
	res = hid_get_manufacturer_string(mpHandle, wstr, MAX_STR);
    sprintf(tmp, "-> CP: mp_driver.panel_open: Manufacturer String %ls\n", wstr);
	XPLMDebugString(tmp);

	hid_set_nonblocking(mpHandle, 1);
	res = hid_read(mpHandle, mp_in_buf, MP_IN_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: mp_driver.panel_open: Error: %ls\n", hid_error(mpHandle));
		XPLMDebugString(tmp);
	}
	res = hid_send_feature_report(mpHandle, mp_zero_panel, MP_OUT_BUF_SIZE);
	if (res < 0) {
	    sprintf(tmp, "-> CP: mp_driver.panel_open: Error: %ls\n", hid_error(mpHandle));
		XPLMDebugString(tmp);
	}
	return 0;
}

int mp_panel_write(unsigned char buf[]) {
	int res = 0;
	int i = 0;
	unsigned char tmpBuffer[MP_OUT_BUF_SIZE];
	for (i = 0; i < MP_OUT_BUF_SIZE; i++) {
		tmpBuffer[i] = buf[i];
	}
	if (mpHandle) {
		res = hid_send_feature_report(mpHandle, tmpBuffer,
				MP_OUT_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: mp_driver.panel_write: Error: %ls\n", hid_error(mpHandle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int mp_panel_read_non_blocking(unsigned char *buf) {
	int res = 0;
	if (mpHandle) {
		hid_set_nonblocking(mpHandle, 1);
		res = hid_read(mpHandle, buf, MP_IN_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: mp_driver.panel_read_non_blocking: Error: %ls\n", hid_error(mpHandle));
			XPLMDebugString(tmp);
		}
	}
	return res;
}

int mp_panel_close() {
	int res = 0;
	if (mpHandle) {
		res = hid_send_feature_report(mpHandle, mp_blank_panel, MP_OUT_BUF_SIZE);
		if (res < 0) {
		    sprintf(tmp, "-> CP: mp_driver.panel_close: Error: %ls\n", hid_error(mpHandle));
			XPLMDebugString(tmp);
		}
		hid_close(mpHandle);
		XPLMDebugString("-> CP: mp_driver.panel_close: panel closed.\n");
		mpHandle = NULL;
	}
	return res;
}

