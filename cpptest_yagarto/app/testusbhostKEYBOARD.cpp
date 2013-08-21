/*
 * testusbhostKEYBOARD.cpp
 *
 *  Created on: 2013Äê8ÔÂ21ÈÕ
 *      Author: Hz
 */

#include "bsp.h"
#include "Usb.h"
#include "hidboot.h"
#include "testusbhostKEYBOARD.h"

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
