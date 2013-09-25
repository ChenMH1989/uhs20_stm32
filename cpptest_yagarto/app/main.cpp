/*
 * main.cpp
 *
 *  Created on: 2013-6-3
 *      Author: Hz
 */

#include "bsp.h"
#include "Usb.h"
#include "hidboot.h"	// todo: will move hid to its testxx file
#include "usbhub.h"

#include "testusbhostKEYBOARD.h"	// HID class test case
#include "testusbhostFAT.h"			// Mass storage class test case
#include "testusbhostSPP.h"			// Bluetooth SPP class test case

USB_OTG_CORE_HANDLE USB_OTG_Core_dev;
USB Usb(&USB_OTG_Core_dev);
USBHub Hub(&Usb);

// HID class, todo : will move hid to its testxx file
HIDBoot<HID_PROTOCOL_KEYBOARD> HidKeyboard(&Usb);
KbdRptParser Prs;

int main(void)
{
	BSP_init();

	STM_EVAL_LEDInit(LED1);		// debug pin
	STM_EVAL_LEDInit(LED2);		// heart, no blink if halted (observed by human eye)
	STM_EVAL_LEDInit(LED3);		// block, no flip if blocked (observed by logic16)

	delay_ms(50);
	STM_EVAL_LEDToggle(LED1);
	delay_ms(50);
	STM_EVAL_LEDToggle(LED1);
	delay_ms(50);
	STM_EVAL_LEDToggle(LED1);

	HidKeyboard.SetReportParser(0, (HIDReportParser*)&Prs);
    // Initialize generic storage. This must be done before USB starts.
    InitClassStorage();
    InitClassBtd();

	if (Usb.Init() != -1)
		printf("Usb is initialized.\n");

	uint32_t heart_cnt = 0;

	for(;;) {

		Usb.Task(&USB_OTG_Core_dev);

		check_fatstatus();
		check_btdstatus();

		if(uint8_t inchar = GetKey()) {
			printf("%c", inchar);
			switch(inchar) {
			case 'b':
				demo_directorybrowse();
				break;
			case 'f':
				demo_fileoperation();
				break;
			case 's':
				demo_speedtest();
				break;
			case 'h':
				printf("\r\nCommand list:\r\n");
				printf(" b : demo directory browsing\n");
				printf(" f : demo file operation\n");
				printf(" s : demo file operation speed\n");
			}
		}

		//delay_ms(500);
		if(++heart_cnt > 50000) {
			heart_cnt = 0;
			//STM_EVAL_LEDToggle(LED2);
		}
		//printf("\ntime:%d", millis());
		//STM_EVAL_LEDToggle(LED3);
	}

	return 1;
}

