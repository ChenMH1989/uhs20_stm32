/*
 * testusbhostKEYBOARD.cpp
 *
 *  Created on: 2013Äê8ÔÂ21ÈÕ
 *      Author: Hz
 */

#include "bsp.h"
#include "Usb.h"
#include "testusbhostKEYBOARD.h"

extern USB Usb;

class KbdRptParser : public KeyboardReportParser
{
	void PrintKey(uint8_t mod, uint8_t key);

protected:
	virtual void OnControlKeysChanged(uint8_t before, uint8_t after);

	virtual void OnKeyDown	(uint8_t mod, uint8_t key);
	virtual void OnKeyUp	(uint8_t mod, uint8_t key);
	virtual void OnKeyPressed(uint8_t key);
};

class MouseRptParser : public MouseReportParser
{
protected:
	virtual void OnMouseMove	(MOUSEINFO *mi);
	virtual void OnLeftButtonUp	(MOUSEINFO *mi);
	virtual void OnLeftButtonDown	(MOUSEINFO *mi);
	virtual void OnRightButtonUp	(MOUSEINFO *mi);
	virtual void OnRightButtonDown	(MOUSEINFO *mi);
	virtual void OnMiddleButtonUp	(MOUSEINFO *mi);
	virtual void OnMiddleButtonDown	(MOUSEINFO *mi);
};

HIDBoot<HID_PROTOCOL_KEYBOARD | HID_PROTOCOL_MOUSE>* HidComposite;
HIDBoot<HID_PROTOCOL_KEYBOARD>* HidKeyboard;
//HIDBoot<HID_PROTOCOL_MOUSE>* HidMouse;

KbdRptParser KbdPrs;
MouseRptParser MousePrs;

void InitHid(void)
{
	HidComposite = new HIDBoot<HID_PROTOCOL_KEYBOARD | HID_PROTOCOL_MOUSE>(&Usb);
	HidKeyboard = new HIDBoot<HID_PROTOCOL_KEYBOARD>(&Usb);
	//HidMouse = new HIDBoot<HID_PROTOCOL_MOUSE>(&Usb);

    HidComposite->SetReportParser(0, (HIDReportParser*)&KbdPrs);
    HidComposite->SetReportParser(1,(HIDReportParser*)&MousePrs);
	HidKeyboard->SetReportParser(0, (HIDReportParser*)&KbdPrs);
	//HidMouse->SetReportParser(0, (HIDReportParser*)&MousePrs);

	printf("\r\nHID Library Started");

}

void KbdRptParser::PrintKey(uint8_t m, uint8_t key)
{
    MODIFIERKEYS mod;
    *((uint8_t*)&mod) = m;
    printf((mod.bmLeftCtrl   == 1) ? "C" : " ");
    printf((mod.bmLeftShift  == 1) ? "S" : " ");
    printf((mod.bmLeftAlt    == 1) ? "A" : " ");
    printf((mod.bmLeftGUI    == 1) ? "G" : " ");

    printf("\n >");
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
};

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

};

void KbdRptParser::OnKeyUp(uint8_t mod, uint8_t key)
{
    //printf("\nUP ");
    PrintKey(mod, key);
};

void KbdRptParser::OnKeyPressed(uint8_t key)
{
    printf("\nASCII: %c", (char)key);
};


void MouseRptParser::OnMouseMove(MOUSEINFO *mi)
{
	printf("\ndx=%d, dy=%d", mi->dX, mi->dY);
};
void MouseRptParser::OnLeftButtonUp	(MOUSEINFO *mi)
{
    printf("\nL Butt Up");
};
void MouseRptParser::OnLeftButtonDown	(MOUSEINFO *mi)
{
	printf("\nL Butt Dn");
};
void MouseRptParser::OnRightButtonUp	(MOUSEINFO *mi)
{
	printf("\nR Butt Up");
};
void MouseRptParser::OnRightButtonDown	(MOUSEINFO *mi)
{
	printf("\nR Butt Dn");
};
void MouseRptParser::OnMiddleButtonUp	(MOUSEINFO *mi)
{
	printf("\nM Butt Up");
};
void MouseRptParser::OnMiddleButtonDown	(MOUSEINFO *mi)
{
	printf("\nM Butt Dn");
};
