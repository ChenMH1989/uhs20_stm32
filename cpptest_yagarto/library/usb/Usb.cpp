/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */
/* USB functions */

//#include "avrpins.h"
//#include "max3421e.h"
#include "usbhost.h"
#include "Usb.h"
#include "bsp.h"

static uint8_t usb_error = 0;
static uint8_t usb_task_state;

/* constructor */
USB::USB(USB_OTG_CORE_HANDLE *pDev) : STM32F207(pDev), bmHubPre(0) {
        usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE; //set up state machine
        init();
}

/* Initialize data structures */
void USB::init() {
        //devConfigIndex = 0;
        bmHubPre = 0;
}

uint8_t USB::getUsbTaskState(void) {
        return ( usb_task_state);
}

void USB::setUsbTaskState(uint8_t state) {
        usb_task_state = state;
}

EpInfo* USB::getEpInfoEntry(uint8_t addr, uint8_t ep) {
        UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

        if (!p || !p->epinfo)
			return NULL;

        EpInfo *pep = p->epinfo;

        for (uint8_t i = 0; i < p->epcount; i++) {
			if ((pep)->epAddr == ep)
				return pep;

			pep++;
        }
        return NULL;
}

/* set device table entry */

/* each device is different and has different number of endpoints. This function plugs endpoint record structure, defined in application, to devtable */
uint8_t USB::setEpInfoEntry(uint8_t addr, uint8_t epcount, EpInfo* eprecord_ptr) {
        if (!eprecord_ptr)
			return USB_ERROR_INVALID_ARGUMENT;

        UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

        if (!p)
			return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

        p->address = addr;
        p->epinfo = eprecord_ptr;
        p->epcount = epcount;

        return 0;
}

uint8_t USB::SetAddress(uint8_t addr, uint8_t ep, EpInfo **ppep, uint16_t &nak_limit) {
        UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

        if (!p)
			return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

        if (!p->epinfo)
			return USB_ERROR_EPINFO_IS_NULL;

        *ppep = getEpInfoEntry(addr, ep);

        if (!*ppep)
			return USB_ERROR_EP_NOT_FOUND_IN_TBL;

        nak_limit = (0x0001UL << (((*ppep)->bmNakPower > USB_NAK_MAX_POWER) ? USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
        nak_limit--;

        /*
          USBTRACE2("\r\nAddress: ", addr);
          USBTRACE2(" EP: ", ep);
          USBTRACE2(" NAK Power: ",(*ppep)->bmNakPower);
          USBTRACE2(" NAK Limit: ", nak_limit);
          USBTRACE("\r\n");
         */
//        regWr(rPERADDR, addr); //set peripheral address
        
//        uint8_t mode = regRd(rMODE);
        
        //Serial.print("\r\nMode: ");
        //Serial.println( mode, HEX);
        //Serial.print("\r\nLS: ");
        //Serial.println(p->lowspeed, HEX);
        	


        // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise
//        regWr(rMODE, (p->lowspeed) ? mode | bmLOWSPEED | bmHubPre : mode & ~(bmHUBPRE | bmLOWSPEED));

        return 0;
}

/* Control transfer. Sets address, endpoint, fills control packet with necessary data, dispatches control packet, and initiates bulk IN transfer,   */
/* depending on request. Actual requests are defined as inlines                                                                                      */
/* return codes:                */
/* 00       =   success         */

/* 01-0f    =   non-zero HRSLT  */
uint8_t USB::ctrlReq(uint8_t addr, uint8_t ep, uint8_t bmReqType, uint8_t bRequest, uint8_t wValLo, uint8_t wValHi,
        uint16_t wInd, uint16_t total, uint16_t nbytes, uint8_t* dataptr, USBReadParser *p) {
        bool direction = false; //request direction, IN or OUT
        uint8_t rcode;
        SETUP_PKT setup_pkt;
        USB_OTG_CORE_HANDLE *pdev = coreConfig;
        URB_STATE URB_Status = URB_IDLE;

        EpInfo *pep = NULL;
        uint16_t nak_limit = 0;
/* the address are set by HC functions as 0*/
        rcode = SetAddress(addr, ep, &pep, nak_limit);

        if (rcode)
			return rcode;

        direction = ((bmReqType & 0x80) > 0);

        /* fill in setup packet */
        setup_pkt.ReqType_u.bmRequestType = bmReqType;
        setup_pkt.bRequest = bRequest;
        setup_pkt.wVal_u.wValueLo = wValLo;
        setup_pkt.wVal_u.wValueHi = wValHi;
        setup_pkt.wIndex = wInd;
        setup_pkt.wLength = total;

//        bytesWr(rSUDFIFO, 8, (uint8_t*) & setup_pkt); //transfer to setup packet FIFO
        // *pep points to EP0 (HC0-out, HC1-in)
        if(addr != 0) {	// in case hid->setAddr() called, we should reset or reuse hid's addr. suck..
        	if(pdev->host.hc[pep->hcNumOut].dev_addr != addr)
				pdev->host.hc[pep->hcNumOut].dev_addr = addr;
				//USBH_Modify_Channel (pdev, pep->hcNumOut, addr, 0, 0, 0, 0);
			if(pdev->host.hc[pep->hcNumIn].dev_addr != addr)
				pdev->host.hc[pep->hcNumIn].dev_addr = addr;
				//USBH_Modify_Channel (pdev, pep->hcNumIn, addr, 0, 0, 0, 0);
        }


        rcode = dispatchPkt(tokSETUP, ep, nak_limit, (uint8_t *)&setup_pkt, sizeof(setup_pkt), pep->hcNumOut); //dispatch packet

        if (rcode) //return HRSLT if not zero
			return ( rcode);

        if (dataptr != NULL) //data stage, if present
        {
			if (direction) //IN transfer
			{
				uint16_t left = total;
#if 1
				//pep->bmRcvToggle = 1; //bmRCVTOG1;
				pdev->host.hc[pep->hcNumIn].toggle_in = 0x1;
				pep->bmRcvToggle = pdev->host.hc[pep->hcNumIn].toggle_in;

				uint16_t read = total;	//nbytes;
				rcode = InTransfer(pep, nak_limit, &read, dataptr);

				// Invoke callback function if inTransfer completed successfully and callback function pointer is specified
				if (!rcode && p)
					((USBReadParser*)p)->Parse(read, dataptr, total - left);

#else
				while (left) {
					// Bytes read into buffer
					uint16_t read = nbytes;
					//uint16_t read = (left<nbytes) ? left : nbytes;

					pdev->host.hc[pep->hcNumIn].toggle_in = 0x1;
					rcode = InTransfer(pep, nak_limit, &read, dataptr);

/*
					if (rcode == hrTOGERR) {
							// yes, we flip it wrong here so that next time it is actually correct!
							pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
							continue;
					}
*/
					if (rcode)
						return rcode;

					// Invoke callback function if inTransfer completed successfully and callback function pointer is specified
					if (!rcode && p)
						((USBReadParser*)p)->Parse(read, dataptr, total - left);

					left -= read;

					if (read < nbytes)
						break;
				}
#endif
                } else //OUT transfer
                {
					//pep->bmSndToggle = 1; //bmSNDTOG1;
					pdev->host.hc[pep->hcNumOut].toggle_out = 0x1;
					pep->bmSndToggle = pdev->host.hc[pep->hcNumOut].toggle_out;
					rcode = OutTransfer(pep, nak_limit, nbytes, dataptr);
			}
			if (rcode) //return error
				return ( rcode);
        }
        // Status stage
        return dispatchPkt((direction) ? tokOUTHS : tokINHS, ep, nak_limit, NULL, 0, (direction) ? pep->hcNumOut : pep->hcNumIn);	//GET if direction
}

/**
  * @brief  USBH_InterruptReceiveData
  *         Receives the Device Response to the Interrupt IN token
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer in which the response needs to be copied
  * @param  length: Length of the data to be received
  * @param  hc_num: Host channel Number
  * @retval Status.
  */
USBH_Status USB::USBH_InterruptReceiveData(uint8_t *buff, uint8_t length, uint8_t hc_num)
{
//TODO: merge this function with InTransfer
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

  pdev->host.hc[hc_num].ep_is_in = 1;
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = length;



  if(pdev->host.hc[hc_num].toggle_in == 0)
  {
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
  }
  else
  {
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
  }

  /* toggle DATA PID */
  pdev->host.hc[hc_num].toggle_in ^= 1;

  HCD_SubmitRequest (pdev , hc_num);

  return USBH_OK;
}

/* IN transfer to arbitrary endpoint. Assumes PERADDR is set. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Keep sending INs and writes data to memory area pointed by 'data'                                                           */

/* rcode 0 if no errors. rcode 01-0f is relayed from dispatchPkt(). Rcode f0 means RCVDAVIRQ error,
            fe USB xfer timeout */
uint8_t USB::inTransfer(uint8_t addr, uint8_t ep, uint16_t *nbytesptr, uint8_t* data) {
        EpInfo *pep = NULL;
        uint16_t nak_limit = 0;
    	USB_OTG_CORE_HANDLE *pdev = coreConfig;

        uint8_t rcode = SetAddress(addr, ep, &pep, nak_limit);

        if (rcode) {
			//printf("SetAddress Failed");
			return rcode;
        }

        return InTransfer(pep, nak_limit, nbytesptr, data);
}

uint8_t USB::InTransfer(EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t* data) {
	uint8_t rcode = 0;
	uint8_t pktsize;

	uint16_t nbytes = *nbytesptr;
	//printf("Requesting %i bytes ", nbytes);
	uint16_t maxpktsize = pep->maxPktSize;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;
	uint32_t hcnum = pep->hcNumIn;	//pdev->host.hc_num_in;

#if 0	// get all packets via rx fifo
	pdev->host.hc[hcnum].data_pid = (pdev->host.hc[hcnum].toggle_in) ? HC_PID_DATA1 : HC_PID_DATA0;
	pdev->host.hc[hcnum].ep_is_in = 1;
	pdev->host.hc[hcnum].xfer_buff = data;
	pdev->host.hc[hcnum].xfer_len = nbytes;

	HCD_SubmitRequest(pdev, hcnum);
#else	// get one packet per transfer.
	//*nbytesptr = 0;	// 1. means how many bytes was received.
						// 2. on stm32, we receive all data at once, so we don't need to count this var.

	if(maxpktsize != pdev->host.hc[hcnum].max_packet)
		pdev->host.hc[hcnum].max_packet = maxpktsize;
	uint8_t ep_addr = pep->epAddr & 0x7F;	// todo: dont like this, will remove "&0x7f"
	if(ep_addr != pdev->host.hc[hcnum].ep_num)
		pdev->host.hc[hcnum].ep_num = ep_addr;
    	//USBH_Modify_Channel(pdev, hcnum, 0, ep_addr, 0, 0, 0);
	if(pep->bmRcvToggle != pdev->host.hc[hcnum].toggle_in)	// for hid composite
		pdev->host.hc[hcnum].toggle_in = pep->bmRcvToggle;

	while (1) // use a 'return' to exit this loop
	{
		rcode = dispatchPkt(tokIN, pep->epAddr, nak_limit, data, nbytes, hcnum); //IN packet to EP-'endpoint'. Function takes care of NAKS.
		if (rcode == hrTOGERR) {
			// yes, we flip it wrong here so that next time it is actually correct!
//			pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
//			regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); //set toggle value
			STM_EVAL_LEDToggle(LED3);
			//pdev->host.hc[hcnum].toggle_in ^= 0x1;	//btd hci case
			printf("\nInXfer - toggle err, hc num=%d", hcnum);	// will meet toggle error here? todo: sometimes once unplugged device, there is small chance that Poll still works here.
			//continue;
		}
		if (rcode) {
			//printf("\r\n>>>>>>>> Problem! dispatchPkt %d", rcode);
			break; //should be 0, indicating ACK. Else return error code.
		}
		/* check for RCVDAVIRQ and generate error if not present */
		/* the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
//		if ((regRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
			//printf(">>>>>>>> Problem! NO RCVDAVIRQ!\r\n");
//			rcode = 0xf0; //receive error
//			break;
//		}
//		pktsize = regRd(rRCVBC); //number of received bytes
		//printf("Got %i bytes \r\n", pktsize);
		// This would be OK, but...
		//assert(pktsize <= nbytes);
//		if (pktsize > nbytes) {
			// This can happen. Use of assert on Arduino locks up the Arduino.
			// So I will trim the value, and hope for the best.
			//printf(">>>>>>>> Problem! Wanted %i bytes but got %i.\r\n", nbytes, pktsize);
//			pktsize = nbytes;
//		}

//		int16_t mem_left = (int16_t)nbytes - *((int16_t*)nbytesptr);

//		if (mem_left < 0)
//			mem_left = 0;

//		data = bytesRd(rRCVFIFO, ((pktsize > mem_left) ? mem_left : pktsize), data);

//		regWr(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer
//		*nbytesptr += pktsize; // add this packet's byte count to total transfer length

		/* The transfer is complete under two conditions:           */
		/* 1. The device sent a short packet (L.T. maxPacketSize)   */
		/* 2. 'nbytes' have been transferred.                       */
		//if ((pktsize < maxpktsize) || (*nbytesptr >= nbytes)) // have we transferred 'nbytes' bytes?
		{	// thanks to large fifo on stm32, we don't need packet by packet handling.
			// Save toggle value
			// the toggle is flipped at interrupt handler.
			//pep->bmRcvToggle = ((regRd(rHRSL) & bmRCVTOGRD)) ? 1 : 0;
			pep->bmRcvToggle = pdev->host.hc[hcnum].toggle_in;	//save last toggle
			//printf("\r\n");

			rcode = 0;
			break;
		} // if
	} //while( 1 )
#endif		
	return ( rcode);
}

/* OUT transfer to arbitrary endpoint. Handles multiple packets if necessary. Transfers 'nbytes' bytes. */
/* Handles NAK bug per Maxim Application Note 4000 for single buffer transfer   */

/* rcode 0 if no errors. rcode 01-0f is relayed from HRSL                       */
uint8_t USB::outTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t* data) {
        EpInfo *pep = NULL;
        uint16_t nak_limit = 0;

        uint8_t rcode = SetAddress(addr, ep, &pep, nak_limit);

        if (rcode)
			return rcode;

        return OutTransfer(pep, nak_limit, nbytes, data);
}

uint8_t USB::OutTransfer(EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data) {
	uint8_t rcode = hrSUCCESS, retry_count;
	uint8_t *data_p = data; //local copy of the data pointer
	uint16_t bytes_tosend, nak_count;
	uint16_t bytes_left = nbytes, last_bytesleft = nbytes;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;
	uint32_t hcnum = pep->hcNumOut;

	uint8_t maxpktsize = pep->maxPktSize;

	if (maxpktsize < 1 || maxpktsize > 64)
		return USB_ERROR_INVALID_MAX_PKT_SIZE;

	if(maxpktsize != pdev->host.hc[hcnum].max_packet)
			pdev->host.hc[hcnum].max_packet = maxpktsize;
#if 0
	pdev->host.hc[hcnum].ep_is_in = 0;
	pdev->host.hc[hcnum].xfer_buff = data; //buff;
	pdev->host.hc[hcnum].xfer_len = nbytes;	//length;

	if(nbytes == 0)
	{ /* For Status OUT stage, Length==0, Status Out PID = 1 */
		pdev->host.hc[hcnum].toggle_out = 1;
	}

	/* Set the Data Toggle bit as per the Flag */
	if ( pdev->host.hc[hcnum].toggle_out == 0)
	{ /* Put the PID 0 */
		pdev->host.hc[hcnum].data_pid = HC_PID_DATA0;
	}
	else
	{ /* Put the PID 1 */
		pdev->host.hc[hcnum].data_pid = HC_PID_DATA1 ;
	}

	HCD_SubmitRequest (pdev, hcnum);
#else
        unsigned long timeout = millis() + USB_XFER_TIMEOUT;

        //regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); //set toggle value
    	pdev->host.hc[hcnum].ep_is_in = 0;
    	pdev->host.hc[hcnum].xfer_buff = data; //buff;
    	pdev->host.hc[hcnum].xfer_len = nbytes;	//length;

    	if(nbytes == 0)
    	{ /* For Status OUT stage, Length==0, Status Out PID = 1 */
    		pdev->host.hc[hcnum].toggle_out = 1;
    	}

    	/* Set the Data Toggle bit as per the Flag */
    	if ( pdev->host.hc[hcnum].toggle_out == 0)
    	{ /* Put the PID 0 */
    		pdev->host.hc[hcnum].data_pid = HC_PID_DATA0;
    	}
    	else
    	{ /* Put the PID 1 */
    		pdev->host.hc[hcnum].data_pid = HC_PID_DATA1 ;
    	}

		retry_count = 0;
		nak_count = 0;
		while (bytes_left) {
			// we send all data at once due to large fifo
			//bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
			//bytesWr(rSNDFIFO, bytes_tosend, data_p); //filling output FIFO
			//regWr(rSNDBC, bytes_tosend); //set number of bytes
			//regWr(rHXFR, (tokOUT | pep->epAddr)); //dispatch packet
			HCD_SubmitRequest(pdev, hcnum);
PollStatus:
			rcode = USB_ERROR_TRANSFER_TIMEOUT;
			while (timeout > millis())  //while (!(regRd(rHIRQ) & bmHXFRDNIRQ)); //wait for the completion IRQ
			{
				rcode = HCD_GetURB_State(pdev, hcnum);	//regRd(rHIRQ);
				if (rcode != URB_IDLE) {	//& bmHXFRDNIRQ) {
					//regWr(rHIRQ, bmHXFRDNIRQ); //clear IRQ
					rcode = 0x00;
					break;
				}
			}
			if(rcode != 0x00)
				return rcode;	// todo: return for what

			rcode = HCD_GetHCState(pdev, hcnum);	//(regRd(rHRSL) & 0x0f);
			//while (rcode && (timeout > millis())) {	// we don't need pulling here because we already did it above
				switch (rcode) {
					case hrNAK:
						nak_count++;	//maybe a NOT_READY happens
						if (nak_limit && (nak_count == nak_limit))
							goto breakout;
/*						else if (URB_NOTREADY == HCD_GetURB_State(pdev, hcnum)) {
							USB_OTG_HCCHAR_TypeDef hcchar;
							hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[hcnum]->HCCHAR);
							if(hcchar.b.eptype == EP_TYPE_BULK) {
								pdev->host.URB_State[hcnum] = URB_IDLE;
								// re-activate the channel
								hcchar.b.chen = 1;
								hcchar.b.chdis = 0;
								USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hcnum]->HCCHAR, hcchar.d32);
			                    STM_EVAL_LEDToggle(LED1);
								goto PollStatus;	//return ( rcode);
							}
						}*/
						break;
					case hrTIMEOUT:
						retry_count++;
						if (retry_count == USB_RETRY_LIMIT)
							goto breakout;
						//return ( rcode);
						break;
					case hrTOGERR:
						// yes, we flip it wrong here so that next time it is actually correct!
						//pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
						//regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); //set toggle value
						printf("\nOutTransfer - togerr");
						//break;
					default:
						goto breakout;
				}//switch( rcode

				/* process NAK according to Host out NAK bug */
/*				uint32_t *addr;
				uint32_t i;
				for(addr = (uint32_t *)0x50000000, i = 0; i < 0x12; addr++, i++) {
					printf("\naddr(%x) = %x", addr, *addr);
				}*/
				USB_OTG_HCTSIZn_TypeDef hctsiz;
				hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[hcnum]->HCTSIZ);

				bytes_left = hctsiz.b.pktcnt << 6;
				if(last_bytesleft != bytes_left) {
					last_bytesleft = bytes_left;
					pdev->host.hc[hcnum].xfer_buff = data + nbytes - bytes_left;
					pdev->host.hc[hcnum].xfer_len = bytes_left;

					if(((nbytes - bytes_left) >> 6) & 0x1) {	// if sent odd times packets
						pdev->host.hc[hcnum].toggle_out ^= 0x1;
						pdev->host.hc[hcnum].data_pid = (pdev->host.hc[hcnum].toggle_out) ? HC_PID_DATA1 : HC_PID_DATA0;
					}
					pdev->host.hc[hcnum].isEvenTimesToggle = 0;	// will be re-calculate in HCD_SubmitRequest
					retry_count = 0;
					nak_count = 0;
				}
				//regWr(rSNDBC, 0);
				//regWr(rSNDFIFO, *data_p);
				//regWr(rSNDBC, bytes_tosend);
				//regWr(rHXFR, (tokOUT | pep->epAddr)); //dispatch packet
				//while (!(regRd(rHIRQ) & bmHXFRDNIRQ)); //wait for the completion IRQ
				//regWr(rHIRQ, bmHXFRDNIRQ); //clear IRQ
				//rcode = (regRd(rHRSL) & 0x0f);
			//}//while( rcode && ....
			// bytes_left seems never decreasing to 0
			//bytes_left -= bytes_tosend;
			//data_p += bytes_tosend;
		}//while( bytes_left...
breakout:
        //pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 1 : 0; //bmSNDTOG1 : bmSNDTOG0;  //update toggle
#endif
        return ( rcode); //should be 0 in all cases
}
/* dispatch USB packet. Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */

/* return codes 0x00-0x0f are HRSLT( 0x00 being success ), 0xff means timeout                       */
uint8_t USB::dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit, uint8_t *data_p = NULL, uint16_t nbytes = 0, uint8_t hcnum = 0) {
        unsigned long timeout = millis() + USB_XFER_TIMEOUT;
        //unsigned long timeout2 = timeout;
        uint8_t tmpdata;
        uint8_t rcode = hrSUCCESS;
        uint8_t retry_count = 0;
        uint16_t nak_count = 0;
        USB_OTG_CORE_HANDLE *pdev = coreConfig;
        uint8_t pid = 0;

		pdev->host.hc[hcnum].nak_count = 0;
		pdev->host.hc[hcnum].nak_limit = nak_limit;

        while (timeout > millis()) {
			//regWr(rHXFR, (token | ep)); //launch the transfer
        	if(token == tokSETUP) {
				USBH_CtlSendSetup(pdev, data_p, hcnum);
				//timeout2 = millis() + 10;
        	} else {
        		if(token == tokOUTHS) {
        			pdev->host.hc[hcnum].toggle_out = 0x1;
        			pdev->host.hc[hcnum].ep_is_in = 0;
        			pid = HC_PID_DATA1;
        		} else if (token == tokINHS) {
        			pdev->host.hc[hcnum].toggle_in = 0x1;
        			pdev->host.hc[hcnum].ep_is_in = 1;
        			pid = HC_PID_DATA1;
        		} else if (token == tokIN){
        			pdev->host.hc[hcnum].ep_is_in = 1;
        			pid = (pdev->host.hc[hcnum].toggle_in)? HC_PID_DATA1 : HC_PID_DATA0;
        		} else {
        			pdev->host.hc[hcnum].ep_is_in = 0;
        			pid = (pdev->host.hc[hcnum].toggle_out)? HC_PID_DATA1 : HC_PID_DATA0;
        		}/*
        		if(pid == HC_PID_DATA0) {
        			STM_EVAL_LEDToggle(LED1);
        			delay_ms(1);
        			STM_EVAL_LEDToggle(LED1);
        			delay_ms(1);
        			STM_EVAL_LEDToggle(LED1);
        		}
        		if(pid == HC_PID_DATA1) {
        			STM_EVAL_LEDToggle(LED1);
        			delay_ms(1);
					STM_EVAL_LEDToggle(LED1);
					delay_ms(1);
					STM_EVAL_LEDToggle(LED1);
					delay_ms(1);
					STM_EVAL_LEDToggle(LED1);
        		}*/
    			pdev->host.hc[hcnum].data_pid = pid;
    			pdev->host.hc[hcnum].xfer_buff = data_p;
    			pdev->host.hc[hcnum].xfer_len = nbytes;

    			HCD_SubmitRequest(pdev, hcnum);
        	}

			rcode = USB_ERROR_TRANSFER_TIMEOUT;
			while (timeout > millis()) //wait for transfer completion
			{
				tmpdata = HCD_GetURB_State(pdev, hcnum);	//regRd(rHIRQ);
				if (tmpdata != URB_IDLE) {	//& bmHXFRDNIRQ) {
					//regWr(rHIRQ, bmHXFRDNIRQ); //clear the interrupt
					rcode = 0x00;
					break;
				} 
				if (pdev->host.ConnSts == 0) {
					rcode = hrJERR;
					return rcode;
				}
/*				else {	//todo: because in bt case, there are two in ep (int-in, bulk-in), that we need to check hid/msc case either.
					rcode = HCD_GetHCState(pdev, hcnum);
					if(rcode == hrNAK) {
						nak_count++;
						if (nak_limit && (nak_count == nak_limit))
							return (rcode);
						uint8_t eptype = pdev->host.hc[hcnum].ep_type;
						if (eptype == EP_TYPE_INTR) {
							break;
						} else if ((eptype == EP_TYPE_CTRL) || (eptype == EP_TYPE_BULK)) {					      // re-activate the channel
							USB_OTG_HCCHAR_TypeDef hcchar;
							hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[hcnum]->HCCHAR);
							hcchar.b.chen = 1;
							hcchar.b.chdis = 0;
							USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hcnum]->HCCHAR, hcchar.d32);
							delay_us(200);
						}
					}
				}*/
			}

			//if (rcode != 0x00) //exit if timeout
			//        return ( rcode);

            rcode = HCD_GetHCState(pdev, hcnum);	//(regRd(rHRSL) & 0x0f); //analyze transfer result
			switch (rcode) {
				case hrNAK: 	//todo: if timeout above with nak, we need to consider the next xfer.
					nak_count = pdev->host.hc[hcnum].nak_count;
					if (nak_limit && (nak_count == nak_limit))
						return (rcode);
					break;
				case hrTIMEOUT:
					retry_count++;
					if (retry_count == USB_RETRY_LIMIT)
						return (rcode);
					break;
				default:
					return (rcode);
			}

        }//while( timeout > millis()

        return ( rcode);
}

/* USB main task. Performs enumeration/cleanup */
void USB::Task(USB_OTG_CORE_HANDLE *pdev) //USB state machine
{
	uint8_t rcode;
	uint8_t tmpdata;
	static unsigned long delay = 0;
	bool lowspeed = false;

	STM32F2::Task();

	tmpdata = getVbusState();

	/* modify USB task state if Vbus changed */
	switch (tmpdata) {
		case SE1: //illegal state
			usb_task_state = USB_DETACHED_SUBSTATE_ILLEGAL;
			lowspeed = false;
			break;
		case SE0: //disconnected
			if ((usb_task_state & USB_STATE_MASK) != USB_STATE_DETACHED)
					usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;
			lowspeed = false;
			break;
		case LSHOST:
//        if ((usb_task_state & USB_STATE_MASK) == USB_STATE_DETACHED) {
			lowspeed = true;
//        }
		case FSHOST: //attached
			if ((usb_task_state & USB_STATE_MASK) == USB_STATE_DETACHED) {
				delay = millis() + USB_SETTLE_DELAY;
				usb_task_state = USB_ATTACHED_SUBSTATE_SETTLE;
				STM_EVAL_LEDToggle(LED1);
			}
			break;
	}// switch( tmpdata

	for (uint8_t i = 0; i < USB_NUMDEVICES; i++)
		if (devConfig[i])
			rcode = devConfig[i]->Poll();

	switch (usb_task_state) {
		case USB_DETACHED_SUBSTATE_INITIALIZE:
				init();

				for (uint8_t i = 0; i < USB_NUMDEVICES; i++)
					if (devConfig[i])
							rcode = devConfig[i]->Release();

				usb_task_state = USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE;
				break;
		case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE: //just sit here
				break;
		case USB_DETACHED_SUBSTATE_ILLEGAL: //just sit here
				break;
		case USB_ATTACHED_SUBSTATE_SETTLE: //settle time for just attached device
			if (delay < millis())
				usb_task_state = USB_ATTACHED_SUBSTATE_RESET_DEVICE;
			break;
		case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
			STM_EVAL_LEDToggle(LED1);
			//regWr(rHCTL, bmBUSRST); //issue bus reset
			// what if i want to reset the specified device? (need to survey usb hub feature later)
			pdev->host.SofHits = 0;
			HCD_ResetPort();

			//usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE;
			// the HCD_ResetPort() function will take care of a complete bus reset. so skip next phase.
			usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_SOF;
			break;
/*			case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
				if ((regRd(rHCTL) & bmBUSRST) == 0) {
						tmpdata = regRd(rMODE) | bmSOFKAENAB; //start SOF generation
						regWr(rMODE, tmpdata);
						usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_SOF;
						//delay = millis() + 20; //20ms wait after reset per USB spec
				}
				break;*/

		case USB_ATTACHED_SUBSTATE_WAIT_SOF: //todo: change check order
			if(pdev->host.port_need_reset) {
				usb_task_state = USB_ATTACHED_SUBSTATE_RESET_DEVICE;
				pdev->host.port_need_reset = 0;
				printf("\nLS Dev, reset again");
				break;
			}
			STM_EVAL_LEDToggle(LED1);

			//if (regRd(rHIRQ) & bmFRAMEIRQ) {
			if(pdev->host.SofHits) {
				//when first SOF received _and_ 20ms has passed we can continue
				usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_RESET;
				delay = millis() + 20;
			}
			break;
		case USB_ATTACHED_SUBSTATE_WAIT_RESET:
			if (delay < millis()) {
				usb_task_state = USB_STATE_CONFIGURING;
			}
			break;
		case USB_STATE_CONFIGURING:
			STM_EVAL_LEDToggle(LED1);
					//Serial.print("\r\nConf.LS: ");
					//Serial.println(lowspeed, HEX);
			printf("\nTODO:Support all 8 USB pipe?");
			rcode = Configuring(0, 0, lowspeed);

			if (rcode) {
				if (rcode != USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE) {
					usb_error = rcode;
					usb_task_state = USB_STATE_ERROR;
				}
			} else
				usb_task_state = USB_STATE_RUNNING;
			break;
		case USB_STATE_RUNNING:
			break;
		case USB_STATE_ERROR:
				//MAX3421E::Init();
	        AddressPool &addrPool = GetAddressPool();
	        // Get pointer to pseudo device with address 0 assigned
	        UsbDevice *p = addrPool.GetUsbDevicePtr(0);

			USBH_Free_Channel(pdev, p->epinfo->hcNumOut);
			USBH_Free_Channel(pdev, p->epinfo->hcNumIn);
			break;
	} // switch( usb_task_state )
}

uint8_t USB::DefaultAddressing(uint8_t parent, uint8_t port, bool lowspeed) {
        //uint8_t		buf[12];
        uint8_t rcode;
        UsbDevice *p0 = NULL, *p = NULL;

        // Get pointer to pseudo device with address 0 assigned
        p0 = addrPool.GetUsbDevicePtr(0);

        if (!p0)
			return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

        if (!p0->epinfo)
			return USB_ERROR_EPINFO_IS_NULL;

        p0->lowspeed = (lowspeed) ? true : false;

        // Allocate new address according to device class
        uint8_t bAddress = addrPool.AllocAddress(parent, false, port);

        if (!bAddress)
			return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

        p = addrPool.GetUsbDevicePtr(bAddress);

        if (!p)
                return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

        p->lowspeed = lowspeed;

        // Assign new address to the device
        rcode = setAddr(0, 0, bAddress);

        if (rcode) {
			addrPool.FreeAddress(bAddress);
			bAddress = 0;
			return rcode;
        }
        return 0;
};

uint8_t USB::AttemptConfig(uint8_t driver, uint8_t parent, uint8_t port, bool lowspeed) {
	uint8_t rcode = 0;
	//printf("AttemptConfig: parent = %i, port = %i\r\n", parent, port);

	rcode = devConfig[driver]->ConfigureDevice(parent, port, lowspeed);
	if (rcode == USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET) {
		if (parent == 0) {
			// Send a bus reset on the root interface.
			printf("\n\rusb::atte need reset?");
//TODO: but reset			regWr(rHCTL, bmBUSRST); //issue bus reset
			delay_ms(102); // delay 102ms, compensate for clock inaccuracy.
		} else {
			// reset parent port
			devConfig[parent]->ResetHubPort(port);
		}
	}
	rcode = devConfig[driver]->Init(parent, port, lowspeed);
	return rcode;
}

/*
 * This is broken. We need to enumerate differently.
 * It causes major problems with several devices if detected in an unexpected order.
 *
 *
 * Oleg - I wouldn't do anything before the newly connected device is considered sane.
 * i.e.(delays are not indicated for brevity):
 * 1. reset
 * 2. GetDevDescr();
 * 3a. If ACK, continue with allocating address, addressing, etc.
 * 3b. Else reset again, count resets, stop at some number (5?).
 * 4. When max.number of resets is reached, toggle power/fail
 * If desired, this could be modified by performing two resets with GetDevDescr() in the middle - however, from my experience, if a device answers to GDD()
 * it doesn't need to be reset again
 * New steps proposal:
 * 1: get address pool instance. exit on fail
 * 2: pUsb->getDevDescr(0, 0, constBufSize, (uint8_t*)buf). exit on fail.
 * 3: bus reset, 100ms delay
 * 4: set address
 * 5: pUsb->setEpInfoEntry(bAddress, 1, epInfo), exit on fail
 * 6: while (configurations) {
 *              for(each configuration) {
 *                      for (each driver) {
 *                              6a: Ask device if it likes configuration. Returns 0 on OK.
 *                                      If successful, the driver configured device.
 *                                      The driver now owns the endpoints, and takes over managing them.
 *                                      The following will need codes:
 *                                          Everything went well, instance consumed, exit with success.
 *                                          Instance already in use, ignore it, try next driver.
 *                                          Not a supported device, ignore it, try next driver.
 *                                          Not a supported configuration for this device, ignore it, try next driver.
 *                                          Could not configure device, fatal, exit with fail.
 *                      }
 *              }
 *    }
 * 7: for(each driver) {
 *      7a: Ask device if it knows this VID/PID. Acts exactly like 6a, but using VID/PID
 * 8: if we get here, no driver likes the device plugged in, so exit failure.
 *
 */
uint8_t USB::Configuring(uint8_t parent, uint8_t port, bool lowspeed) {
        //uint8_t bAddress = 0;
        //printf("Configuring: parent = %i, port = %i\r\n", parent, port);
        uint8_t devConfigIndex;
        uint8_t rcode = 0;
        uint8_t buf[sizeof (USB_DEVICE_DESCRIPTOR)];
        UsbDevice *p = NULL;
        EpInfo *oldep_ptr = NULL;
        EpInfo epInfo;
        USB_OTG_CORE_HANDLE *pdev = coreConfig;

        epInfo.epAddr = 0;
        epInfo.maxPktSize = 8;
        epInfo.epAttribs = 0;
        epInfo.bmNakPower = USB_NAK_MAX_POWER;
		// assume:
		// HC0 for control - out
        // HC1 for control - in
        //uint8_t hcnum = USBH_GetFreeChannel(pdev);
        //if(hcnum > 1) {
        //	USBH_Free_Channel(pdev, epInfo.hcNumOut);
        //	USBH_Free_Channel(pdev, epInfo.hcNumIn);
        //}
        USBH_Free_Channel(pdev, 0);
        USBH_Free_Channel(pdev, 1);
		epInfo.hcNumOut = USBH_Alloc_Channel(pdev, 0x00);	// ep_addr = 0
		epInfo.hcNumIn = USBH_Alloc_Channel(pdev, 0x80);
		USBH_Open_Channel(pdev, epInfo.hcNumOut, 0x0, (lowspeed)?bmLOWSPEED:bmFULLSPEED, EP_TYPE_CTRL, 0x8);
		USBH_Open_Channel(pdev, epInfo.hcNumIn,	0x0, (lowspeed)?bmLOWSPEED:bmFULLSPEED, EP_TYPE_CTRL, 0x8);
		printf("\nControl Pipe: out = %d (0), in = %d (1)", epInfo.hcNumOut, epInfo.hcNumIn);

        delay_ms(1000);
        AddressPool &addrPool = GetAddressPool();
        // Get pointer to pseudo device with address 0 assigned
        p = addrPool.GetUsbDevicePtr(0);
        if (!p) {
			printf("Configuring error: USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL\r\n");
			return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
        }

        // Save old pointer to EP_RECORD of address 0
//        oldep_ptr = p->epinfo;

        // Temporary assign new pointer to epInfo to p->epinfo in order to
        // avoid toggle inconsistence

        p->epinfo = &epInfo;

        p->lowspeed = lowspeed;

        // Get device descriptor
        rcode = getDevDescr(0, 0, 8, (uint8_t*)buf);	// 8 should be enough, sizeof (USB_DEVICE_DESCRIPTOR)
        printf("\nControl - Got 1st 8 bytes desc");

        // Extract Max Packet Size from the device descriptor
        epInfo.maxPktSize = (uint8_t)((USB_DEVICE_DESCRIPTOR*)buf)->bMaxPacketSize0;
        //USB::USBH_Modify_Channel (pdev, epInfo.hcNumOut, 0, 0, 0, 0, epInfo.maxPktSize);
        //USB::USBH_Modify_Channel (pdev, epInfo.hcNumIn, 0, 0, 0, 0, epInfo.maxPktSize);

        // Restore p->epinfo
//         keep CtrlXfer's hcNumOut/In in p->epinfo. p->epinfo = oldep_ptr;

        if (rcode) {
            printf("Configuring error: Can't get USB_DEVICE_DESCRIPTOR\r\n");
            return rcode;
        }

        // to-do?
        // Allocate new address according to device class
        //bAddress = addrPool.AllocAddress(parent, false, port);

        //if (!bAddress)
        //        return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;
        rcode = getDevDescr(0, 0, sizeof(USB_DEVICE_DESCRIPTOR), (uint8_t*)buf);
        printf("\nControl - Got 2nd 18 bytes desc.");

        uint16_t vid = (uint16_t)((USB_DEVICE_DESCRIPTOR*)buf)->idVendor;
        uint16_t pid = (uint16_t)((USB_DEVICE_DESCRIPTOR*)buf)->idProduct;
        uint8_t klass = ((USB_DEVICE_DESCRIPTOR*)buf)->bDeviceClass;

        // Attempt to configure if VID/PID or device class matches with a driver
        for (devConfigIndex = 0; devConfigIndex < USB_NUMDEVICES; devConfigIndex++) {
                if (!devConfig[devConfigIndex]) continue; // no driver
                if (devConfig[devConfigIndex]->GetAddress()) continue; // consumed
                if (devConfig[devConfigIndex]->VIDPIDOK(vid, pid)) {
					rcode = AttemptConfig(devConfigIndex, parent, port, lowspeed);
					break;
                } else if (devConfig[devConfigIndex]->DEVCLASSOK(klass)) {
					rcode = AttemptConfig(devConfigIndex, parent, port, lowspeed);
					if (!rcode) break;
                }
        }

        if (devConfigIndex < USB_NUMDEVICES) {
                return rcode;
        }


        // blindly attempt to configure
        for (devConfigIndex = 0; devConfigIndex < USB_NUMDEVICES; devConfigIndex++) {
                if (!devConfig[devConfigIndex]) continue;
                if (devConfig[devConfigIndex]->GetAddress()) continue; // consumed
                rcode = AttemptConfig(devConfigIndex, parent, port, lowspeed);

                //printf("ERROR ENUMERATING %2.2x\r\n", rcode);
                if (!(rcode == USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED || rcode == USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE)) {
                        // in case of an error dev_index should be reset to 0
                        //		in order to start from the very beginning the
                        //		next time the program gets here
                        //if (rcode != USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE)
                        //        devConfigIndex = 0;
                        return rcode;
                }
        }
        // if we get here that means that the device class is not supported by any of registered classes
        rcode = DefaultAddressing(parent, port, lowspeed);

        return rcode;
}

uint8_t USB::ReleaseDevice(uint8_t addr) {
        if (!addr)
                return 0;
        USB_OTG_CORE_HANDLE *pdev = coreConfig;
        for (uint8_t i = 0; i < USB_NUMDEVICES; i++) {
                if(!devConfig[i]) continue;
                if (devConfig[i]->GetAddress() == addr)
                        return devConfig[i]->Release();
        }
        return 0;
}

#if 1 //!defined(USB_METHODS_INLINE)
//get device descriptor

uint8_t USB::getDevDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t* dataptr) {
        return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, nbytes, nbytes, dataptr, NULL));
}
//get configuration descriptor

uint8_t USB::getConfDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t conf, uint8_t* dataptr) {
        return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes, nbytes, dataptr, NULL));
}

uint8_t USB::getConfDescr(uint8_t addr, uint8_t ep, uint8_t conf, USBReadParser *p) {
        const uint32_t bufSize = 256;	//64; due to receiving more than 1 packet (64bytes) in stm imple.
        									// we need a large buffer for BTD class, which has a 177 bytes desc.
        uint8_t buf[bufSize];

        uint8_t ret = getConfDescr(addr, ep, 8, conf, buf);

        if (ret)
			return ret;

        uint16_t total = ((USB_CONFIGURATION_DESCRIPTOR*)buf)->wTotalLength;

        //USBTRACE2("\r\ntotal conf.size:", total);

        return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, total, bufSize, buf, p));
}

//get string descriptor

uint8_t USB::getStrDescr(uint8_t addr, uint8_t ep, uint16_t ns, uint8_t index, uint16_t langid, uint8_t* dataptr) {
        return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, langid, ns, ns, dataptr, NULL));
}
//set address

uint8_t USB::setAddr(uint8_t oldaddr, uint8_t ep, uint8_t newaddr) {
        return ( ctrlReq(oldaddr, ep, bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000, 0x0000, NULL, NULL));
}
//set configuration


uint8_t USB::setConf(uint8_t addr, uint8_t ep, uint8_t conf_value) {
        return ( ctrlReq(addr, ep, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000, 0x0000, NULL, NULL));
}

#endif // defined(USB_METHODS_INLINE)

