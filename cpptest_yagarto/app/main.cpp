/*
 * main.cpp
 *
 *  Created on: 2013-6-3
 *      Author: Hz
 */

#include "bsp.h"
#include "Usb.h"
#include "hidboot.h"
#include "usbhub.h"
#include "masstorage.h"

class KbdRptParser : public KeyboardReportParser
{
	void PrintKey(uint8_t mod, uint8_t key);

protected:
	virtual void OnControlKeysChanged(uint8_t before, uint8_t after);

	virtual void OnKeyDown	(uint8_t mod, uint8_t key);
	virtual void OnKeyUp	(uint8_t mod, uint8_t key);
	virtual void OnKeyPressed(uint8_t key);
};

void KbdRptParser::PrintKey(uint8_t m, uint8_t key)
{
    MODIFIERKEYS mod;
    *((uint8_t*)&mod) = m;
    printf((mod.bmLeftCtrl   == 1) ? "C" : " ");
    printf((mod.bmLeftShift  == 1) ? "S" : " ");
    printf((mod.bmLeftAlt    == 1) ? "A" : " ");
    printf((mod.bmLeftGUI    == 1) ? "G" : " ");

    printf(" >");
    //D_PrintHex<uint8_t>(key, 0x80);
    printf("%x", key);
    printf("< ");

    printf((mod.bmRightCtrl   == 1) ? "C" : " ");
    printf((mod.bmRightShift  == 1) ? "S" : " ");
    printf((mod.bmRightAlt    == 1) ? "A" : " ");
    printf((mod.bmRightGUI    == 1) ? "G" : " ");
};

void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
    //printf("\nDN ");
    PrintKey(mod, key);
    uint8_t c = OemToAscii(mod, key);

    if (c)
        OnKeyPressed(c);
}

void KbdRptParser::OnControlKeysChanged(uint8_t before, uint8_t after) {

    MODIFIERKEYS beforeMod;
    *((uint8_t*)&beforeMod) = before;

    MODIFIERKEYS afterMod;
    *((uint8_t*)&afterMod) = after;

    if (beforeMod.bmLeftCtrl != afterMod.bmLeftCtrl) {
        printf("\nLeftCtrl changed");
    }
    if (beforeMod.bmLeftShift != afterMod.bmLeftShift) {
        printf("\nLeftShift changed");
    }
    if (beforeMod.bmLeftAlt != afterMod.bmLeftAlt) {
        printf("\nLeftAlt changed");
    }
    if (beforeMod.bmLeftGUI != afterMod.bmLeftGUI) {
        printf("\nLeftGUI changed");
    }

    if (beforeMod.bmRightCtrl != afterMod.bmRightCtrl) {
        printf("\nRightCtrl changed");
    }
    if (beforeMod.bmRightShift != afterMod.bmRightShift) {
        printf("\nRightShift changed");
    }
    if (beforeMod.bmRightAlt != afterMod.bmRightAlt) {
        printf("\nRightAlt changed");
    }
    if (beforeMod.bmRightGUI != afterMod.bmRightGUI) {
        printf("\nRightGUI changed");
    }

}

void KbdRptParser::OnKeyUp(uint8_t mod, uint8_t key)
{
    //printf("\nUP ");
    PrintKey(mod, key);
}

void KbdRptParser::OnKeyPressed(uint8_t key)
{
    printf("\nASCII: %c", (char)key);
};

USB_OTG_CORE_HANDLE USB_OTG_Core_dev;
USB Usb(&USB_OTG_Core_dev);
USBHub Hub(&Usb);
HIDBoot<HID_PROTOCOL_KEYBOARD> HidKeyboard(&Usb);
KbdRptParser Prs;
//BulkOnly Bulk(&Usb);

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


	if (Usb.Init() != -1)
		printf("Usb is initialized.\n");

	HidKeyboard.SetReportParser(0, (HIDReportParser*)&Prs);

	uint32_t heart_cnt = 0;

	for(;;) {

		Usb.Task(&USB_OTG_Core_dev);

		//delay_ms(500);
		if(++heart_cnt > 50000) {
			heart_cnt = 0;
			STM_EVAL_LEDToggle(LED2);
		}
		//printf("\ntime:%d", millis());
		STM_EVAL_LEDToggle(LED3);
	}

	return 1;
}

