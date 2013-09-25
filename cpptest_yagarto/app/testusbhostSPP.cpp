/*
 * testusbhostSPP.cpp
 *
 *  Created on: 2013Äê9ÔÂ12ÈÕ
 *      Author: Hz
 */

#include <bsp.h>
#include <Usb.h>
#include <SPP.h>

extern USB Usb;

//BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
BTD* Btd;
//SPP* SerialBT[2]; // We will use this pointer to store the two instance, you can easily make it larger if you like, but it will use a lot of RAM!
//const uint8_t length = sizeof(SerialBT)/sizeof(SerialBT[0]); // Get the lenght of the array
//uint8_t firstMessage[length] = { true }; // Set all to true

SPP* SerialBT;

uint8_t buffer[50];

void InitClassBtd(void) {
	Btd = new BTD(&Usb);
	//for(uint32_t i=0;i<length;i++)
		SerialBT = new SPP(Btd); // This will set the name to the default: "Arduino" and the pin to "1234" for all connections

	printf("\r\nSPP Bluetooth Library Started");

}

void check_btdstatus(void) {
	static uint32_t firstMessage = true;

	if(SerialBT->connected) {
		if(firstMessage) {
		  firstMessage = false;
		  uint8_t str[] = "Hello from Arduino";
		  SerialBT->write(str, 18); // Send welcome message
		}
	    if(uint8_t inchar = GetKey())	//if(Serial.available())
	    	SerialBT->write(inchar);
	    if(SerialBT->available()) {
	    	uint8_t outchar = SerialBT->read();
	    	printf("\n->%c", outchar);
	    }

	}
	else
		firstMessage = true;
}
