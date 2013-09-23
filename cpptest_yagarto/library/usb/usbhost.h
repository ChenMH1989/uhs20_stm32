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
/* STM32F2-based USB Host Library header file */
#ifndef _USBHOST_H_
#define _USBHOST_H_
//TODO: this file should be renamed as some thing like stm32fxx.. or usb_bsp
#include "bsp.h"

//#include "avrpins.h"
//#include "max3421e.h"
#include "usb_core.h"
#include "usb_ch9.h"
#include <stdio.h>
#include "usb_defines.h"

#define RX_FIFO_FS_SIZE                          128
// 1. the stm32's library has a bug when deal with more than NPTXFIFOSIZE data
// 2. the uhs20 library handles 512 bytes (128words) for each usb transaction.
#define TXH_NP_FS_FIFOSIZ                        128	// 96
#define TXH_P_FS_FIFOSIZ                         128	// 96

#define USBH_SETUP_PKT_SIZE   8
#define USBH_EP0_EP_NUM       0
#define USBH_MAX_PACKET_SIZE  0x40

#define HOST_POWERSW_PORT_RCC            RCC_AHB1Periph_GPIOH
#define HOST_POWERSW_PORT                GPIOH
#define HOST_POWERSW_VBUS                GPIO_Pin_5



#define bmSE0       0xF0    	// remove.. disconnect state ?
#define bmSE1       0xF1    	// remove.. reserved / illegal state ?

#define bmHIGHSPEED HPRT0_PRTSPD_HIGH_SPEED
#define bmFULLSPEED HPRT0_PRTSPD_FULL_SPEED		// full speed
#define bmLOWSPEED	HPRT0_PRTSPD_LOW_SPEED		// low speed
#define bmHUBPRE    0x04

//TODO: should be replaced
/* Host transfer token values for writing the HXFR register (R30)   */
/* OR this bit field with the endpoint number in bits 3:0               */
#define tokSETUP  0x10  // HS=0, ISO=0, OUTNIN=0, SETUP=1
#define tokIN     0x00  // HS=0, ISO=0, OUTNIN=0, SETUP=0
#define tokOUT    0x20  // HS=0, ISO=0, OUTNIN=1, SETUP=0
#define tokINHS   0x80  // HS=1, ISO=0, OUTNIN=0, SETUP=0
#define tokOUTHS  0xA0  // HS=1, ISO=0, OUTNIN=1, SETUP=0
#define tokISOIN  0x40  // HS=0, ISO=1, OUTNIN=0, SETUP=0
#define tokISOOUT 0x60  // HS=0, ISO=1, OUTNIN=1, SETUP=0
/* Host error result codes, the 4 LSB's in the HRSL register */
#define hrSUCCESS   0x00
#define hrBUSY      0x01
#define hrBADREQ    0x02
#define hrUNDEF     0x03
#define hrNAK       0x04
#define hrSTALL     0x05
#define hrTOGERR    0x06
#define hrWRONGPID  0x07
#define hrBADBC     0x08
#define hrPIDERR    0x09
#define hrPKTERR    0x0A
#define hrCRCERR    0x0B
#define hrKERR      0x0C
#define hrJERR      0x0D
#define hrTIMEOUT   0x0E
#define hrBABBLE    0x0F

#define SE0     0
#define SE1     1
#define FSHOST  2
#define LSHOST  3

#define HC_MAX           8

#define HC_OK            0x0000
#define HC_USED          0x8000
#define HC_ERROR         0xFFFF
#define HC_USED_MASK     0x7FFF

typedef enum {
  USBH_OK   = 0,
  USBH_BUSY,
  USBH_FAIL,
  USBH_NOT_SUPPORTED,
  USBH_UNRECOVERED_ERROR,
  USBH_ERROR_SPEED_UNKNOWN,
  USBH_APPLY_DEINIT
}USBH_Status;


template< typename SS, typename INTR > class STM32F2 {
        static uint8_t vbusState;

public:
        USB_OTG_CORE_HANDLE *coreConfig;

        STM32F2(USB_OTG_CORE_HANDLE *pDev);
        void regWr(uint8_t reg, uint8_t data);
        uint8_t* bytesWr(uint8_t reg, uint8_t nbytes, uint8_t* data_p);
        void gpioWr(uint8_t data);
        uint8_t regRd(uint8_t reg);
        uint8_t* bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p);
        uint8_t gpioRd();
        uint16_t reset();
        int8_t Init();

        uint8_t getVbusState(void) {
                return vbusState;
        };
/*
        USB_OTG_CORE_HANDLE& GetCoreConfig() {
        	return (USB_OTG_CORE_HANDLE&)coreConfig;
        }
*/
        void busprobe();
        uint8_t GpxHandler();
        uint8_t IntHandler();
        uint32_t Task();
        uint32_t HCD_ResetPort(void);

        static uint8_t USB_OTG_IsHostMode(USB_OTG_CORE_HANDLE *pdev);
        static uint32_t USB_OTG_ReadCoreItr(USB_OTG_CORE_HANDLE *pdev);
        static USB_OTG_STS USB_OTG_WritePacket(USB_OTG_CORE_HANDLE *pdev, uint8_t *src, uint8_t ch_ep_num, uint16_t len);
        static void * USB_OTG_ReadPacket(USB_OTG_CORE_HANDLE *pdev, uint8_t *dest, uint16_t len);
		static uint32_t USB_OTG_GetMode(USB_OTG_CORE_HANDLE *pdev);
		static USB_OTG_STS USB_OTG_HC_Halt(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
		static uint32_t USB_OTG_ReadHostAllChannels_intr (USB_OTG_CORE_HANDLE *pdev);
        static void USB_OTG_InitFSLSPClkSel(USB_OTG_CORE_HANDLE *pdev, uint8_t freq);
        static uint32_t USB_OTG_ResetPort(USB_OTG_CORE_HANDLE *pdev);
        static USB_OTG_STS USB_OTG_HC_DoPing(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
        static uint32_t USB_OTG_ReadHPRT0(USB_OTG_CORE_HANDLE *pdev);

        static uint8_t USBH_Alloc_Channel(USB_OTG_CORE_HANDLE *pdev, uint8_t ep_addr);
        static uint16_t USBH_GetFreeChannel(USB_OTG_CORE_HANDLE *pdev);
        static uint8_t USBH_Open_Channel(USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num, uint8_t dev_address, uint8_t speed, uint8_t ep_type, uint16_t mps);
        static uint8_t USBH_Modify_Channel (USB_OTG_CORE_HANDLE *pdev, uint8_t hc_num, uint8_t dev_address, uint8_t speed, uint8_t ep_type, uint16_t mps);
        static USB_OTG_STS USB_OTG_HC_Init(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
        static USBH_Status USBH_Free_Channel(USB_OTG_CORE_HANDLE *pdev, uint8_t idx);
        static USBH_Status USBH_CtlSendSetup(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff, uint8_t hc_num);
        static USBH_Status USBH_InterruptReceiveData(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff, uint8_t length, uint8_t hc_num);
        static uint32_t HCD_SubmitRequest(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
        static USB_OTG_STS USB_OTG_HC_StartXfer(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num);
        static uint8_t USB_OTG_IsEvenFrame (USB_OTG_CORE_HANDLE *pdev);
        static uint32_t HCD_GetCurrentFrame (USB_OTG_CORE_HANDLE *pdev);
        static URB_STATE HCD_GetURB_State (USB_OTG_CORE_HANDLE *pdev , uint8_t ch_num);
        static uint8_t HCD_GetHCState (USB_OTG_CORE_HANDLE *pdev,  uint8_t ch_num) ;

private:
        void USB_OTG_BSP_Init(void);
        uint32_t HCD_Init(USB_OTG_CORE_ID_TypeDef coreID);
        USB_OTG_STS USB_OTG_SelectCore(USB_OTG_CORE_ID_TypeDef coreID);
        USB_OTG_STS USB_OTG_CoreInit(void);
        USB_OTG_STS USB_OTG_CoreReset(void);
        void USB_OTG_BSP_EnableInterrupt(void);
        USB_OTG_STS USB_OTG_DisableGlobalInt(void);
        USB_OTG_STS USB_OTG_SetCurrentMode(uint32_t mode);
        USB_OTG_STS USB_OTG_CoreInitHost(void);
        USB_OTG_STS USB_OTG_FlushTxFifo(uint32_t num);
        USB_OTG_STS USB_OTG_FlushRxFifo(void);
        void USB_OTG_DriveVbus(uint8_t state);
        USB_OTG_STS USB_OTG_EnableHostInt(void);
        void USB_OTG_EnableCommonInt(void);
        USB_OTG_STS USB_OTG_EnableGlobalInt(void);
        void USB_OTG_BSP_ConfigVBUS(void);
        void USB_OTG_BSP_DriveVBUS(uint8_t state);
};

template< typename SS, typename INTR >
        uint8_t STM32F2< SS, INTR >::vbusState = 0;

/* constructor */
template< typename SS, typename INTR >
STM32F2< SS, INTR >::STM32F2(USB_OTG_CORE_HANDLE *pDev) : coreConfig(pDev) {
	/* ------- 1. Hardware Init ------- */
	USB_OTG_BSP_Init();

	/* ------- 2. configure GPIO pin used for switching VBUS power ------- */
	USB_OTG_BSP_ConfigVBUS();

	/* ------- 3. Host de-initializations ------- */
	//USBH_DeInit(pdev, phost);
//	USBH_Free_Channel(pDev, pDev->host.hc_num_out);
//	USBH_Free_Channel(pDev, pDev->host.hc_num_in);

	/* ------- 4.Register class and user callbacks ------- */
	//phost->class_cb = class_cb;
	//phost->usr_cb = usr_cb;

	/* ------- 5. Start the USB OTG core ------- */
	HCD_Init(USB_OTG_FS_CORE_ID);

	/* ------- 6. Upon Init call usr call back ------- */
	//phost->usr_cb->Init();

	/* ------- 7. Enable Interrupts */
	USB_OTG_BSP_EnableInterrupt();
};

/* write single byte into MAX3421 register */
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::regWr(uint8_t reg, uint8_t data) {
/*        SS::Clear();
        SPDR = (reg | 0x02);
        while(!(SPSR & (1 << SPIF)));
        SPDR = data;
        while(!(SPSR & (1 << SPIF)));
        SS::Set();*/
        return;
};
/* multiple-byte write                            */

/* returns a pointer to memory position after last written */
template< typename SS, typename INTR >
uint8_t* STM32F2< SS, INTR >::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t* data_p) {
/*        SS::Clear();
        SPDR = (reg | 0x02); //set WR bit and send register number
        while(nbytes--) {
                while(!(SPSR & (1 << SPIF))); //check if previous byte was sent
                SPDR = (*data_p); // send next data byte
                data_p++; // advance data pointer
        }
        while(!(SPSR & (1 << SPIF)));
        SS::Set();*/
        return( data_p);
}
/* GPIO write                                           */
/*GPIO byte is split between 2 registers, so two writes are needed to write one byte */

/* GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2 */
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::gpioWr(uint8_t data) {
/*        regWr(rIOPINS1, data);
        data >>= 4;
        regWr(rIOPINS2, data);*/
        return;
}

/* single host register read    */
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::regRd(uint8_t reg) {
/*        SS::Clear();
        SPDR = reg;
        while(!(SPSR & (1 << SPIF)));
        SPDR = 0; //send empty byte
        while(!(SPSR & (1 << SPIF)));
        SS::Set();*/
        return( reg);
}
/* multiple-byte register read  */

/* returns a pointer to a memory position after last read   */
template< typename SS, typename INTR >
uint8_t* STM32F2< SS, INTR >::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t* data_p) {
/*        SS::Clear();
        SPDR = reg;
        while(!(SPSR & (1 << SPIF))); //wait
        while(nbytes) {
                SPDR = 0; //send empty byte
                nbytes--;
                while(!(SPSR & (1 << SPIF)));
#if 0
                {
                        *data_p = SPDR;
                        printf("%2.2x ", *data_p);
                }
                data_p++;
        }
        printf("\r\n");
#else
                *data_p++ = SPDR;
        }
#endif
        SS::Set();*/
        return( data_p);
}
/* GPIO read. See gpioWr for explanation */

/* GPIN pins are in high nibbles of IOPINS1, IOPINS2    */
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::gpioRd() {
        uint8_t gpin = 0;
/*        gpin = regRd(rIOPINS2); //pins 4-7
        gpin &= 0xf0; //clean lower nibble
        gpin |= (regRd(rIOPINS1) >> 4); //shift low bits and OR with upper from previous operation.*/
        return( gpin);
}

/* reset ???. Returns number of cycles it took for PLL to stabilize after reset
  or zero if PLL haven't stabilized in 65535 cycles */
template< typename SS, typename INTR >
uint16_t STM32F2< SS, INTR >::reset() {
//        uint16_t i = 0;
//        regWr(rUSBCTL, bmCHIPRES);
//        regWr(rUSBCTL, 0x00);
//        while(++i) {
//                if((regRd(rUSBIRQ) & bmOSCOKIRQ)) {
//                        break;
//                }
//        }
//        return( i);
	// reset usb phy? is that necessary? just keep the code (above) here...
	return 1;
}

/* initialize STM32F2. Set Host mode, pullups, and stuff. Returns 0 if success, -1 if not */
// it seems we have already done this init function in HCD_Init in USB::STM32F2
template< typename SS, typename INTR >
int8_t STM32F2< SS, INTR >::Init() {
	if(reset() == 0) { //OSCOKIRQ hasn't asserted in time
		return( -1);
	}
	//regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host

	//regWr(rHIEN, bmCONDETIE | bmFRAMEIE); //connection detection

	// check if device is connected
	//regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
	//while(!(regRd(rHCTL) & bmSAMPLEBUS)); //wait for sample operation to finish

	busprobe(); //check if anything is connected

	//regWr(rHIRQ, bmCONDETIRQ); //clear connection detect interrupt
	//regWr(rCPUCTL, 0x01); //enable interrupt pin

	return( 0);
}

/* probe bus to determine device presence and speed and switch host to this speed */
// currently, I haven't make any host switching here.
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::busprobe() {
	USB_OTG_HPRT0_TypeDef hprt0;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;
	uint32_t bus_sample;    //uint8_t bus_sample;

	hprt0.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
	if(pdev->host.ConnSts) {
		bus_sample = hprt0.b.prtspd;
	} else {
		bus_sample = bmSE0;
	}
    //bus_sample = regRd(rHRSL); //Get J,K status STM32F2: J/K means Pos/Neg for D+/D-
    //bus_sample &= (bmJSTATUS | bmKSTATUS); //zero the rest of the byte
	// ok, I haven't found any information about D+/D- alternation detect.
	// so, here we assume the correct d+/d- are connected.
	switch(bus_sample) { //start full-speed or low-speed host
		case(bmFULLSPEED):
			vbusState = FSHOST;
			break;
		case(bmLOWSPEED):
			vbusState = LSHOST;
			break;
	/*
		case( bmJSTATUS):
				if((regRd(rMODE) & bmLOWSPEED) == 0) {
						regWr(rMODE, MODE_FS_HOST); //start full-speed host
						vbusState = FSHOST;
				} else {
						regWr(rMODE, MODE_LS_HOST); //start low-speed host
						vbusState = LSHOST;
				}
				break;
		case( bmKSTATUS):
				if((regRd(rMODE) & bmLOWSPEED) == 0) {
						regWr(rMODE, MODE_LS_HOST); //start low-speed host
						vbusState = LSHOST;
				} else {
						regWr(rMODE, MODE_FS_HOST); //start full-speed host
						vbusState = FSHOST;
				}
				break;
		case( bmSE1): //illegal state
				vbusState = SE1;
				break; */
		case( bmSE0): //disconnected state
			//	regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ);
			// we should already tidy up in disconnected isr.
			vbusState = SE0;
			break;
		default:
			vbusState = SE1;

	}//end switch( bus_sample )

}

/* MAX3421 state change task and interrupt handler */
// all usb phy state and interrupt matters are handled by int handler.
// so we should no longer need this function.
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::Task(void) {
	uint32_t rcode = 0;
//    uint8_t pinvalue;

	//USB_HOST_SERIAL.print("Vbus state: ");
	//USB_HOST_SERIAL.println( vbusState, HEX );
//        pinvalue = 000;	//INTR::IsSet(); //Read();
//        if(pinvalue == 0) {
//			rcode = IntHandler();
//        }
	// it seems only connection status is cared.
	// and we have no necessary to lookup connected devices all the time.
	busprobe();

	return(rcode);
}

template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::IntHandler() {

        uint8_t HIRQ;
        uint8_t HIRQ_sendback = 0x00;
/*
        HIRQ = regRd(rHIRQ); //determine interrupt source
        //if( HIRQ & bmFRAMEIRQ ) {               //->1ms SOF interrupt handler
        //    HIRQ_sendback |= bmFRAMEIRQ;
        //}//end FRAMEIRQ handling
        if(HIRQ & bmCONDETIRQ) {	// max3421, a peripheral is attached.
                busprobe();
                HIRQ_sendback |= bmCONDETIRQ;
        }
        // End HIRQ interrupts handling, clear serviced IRQs
        regWr(rHIRQ, HIRQ_sendback);
*/
        return( HIRQ_sendback);
}


/**
  * @brief  HCD_ResetPort
  *         Issues the reset command to device
  * @param  pdev : Selected device
  * @retval Status
  */
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::HCD_ResetPort(void)
{
  /*
  Before starting to drive a USB reset, the application waits for the OTG
  interrupt triggered by the debounce done bit (DBCDNE bit in OTG_FS_GOTGINT),
  which indicates that the bus is stable again after the electrical debounce
  caused by the attachment of a pull-up resistor on DP (FS) or DM (LS).
  */
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

	USB_OTG_ResetPort(pdev);
	return 0;
}

/* ------- static function members ------- */

/**
* @brief  USB_OTG_IsHostMode : Check if it is host mode
* @param  pdev : Selected device
* @retval num_in_ep
*/
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::USB_OTG_IsHostMode(USB_OTG_CORE_HANDLE *pdev)
{
	return (USB_OTG_GetMode(pdev) == HOST_MODE);
}

/**
* @brief  USB_OTG_ReadCoreItr : returns the Core Interrupt register
* @param  pdev : Selected device
* @retval Status
*/
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::USB_OTG_ReadCoreItr(USB_OTG_CORE_HANDLE *pdev)
{
	uint32_t v = 0;

	v = USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTSTS);
	v &= USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTMSK);

	return v;
}


/**
* @brief  USB_OTG_WritePacket : Writes a packet into the Tx FIFO associated
*         with the EP
* @param  pdev : Selected device
* @param  src : source pointer
* @param  ch_ep_num : end point number
* @param  bytes : No. of bytes
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_WritePacket(USB_OTG_CORE_HANDLE *pdev,
								uint8_t             *src,
                                uint8_t             ch_ep_num,
                                uint16_t            len)
{
	USB_OTG_STS status = USB_OTG_OK;
	uint32_t a;
  if (pdev->cfg.dma_enable == 0)
  {
    uint32_t count32b= 0 , i= 0;
    __IO uint32_t *fifo;

    count32b =  (len + 3) / 4;
    fifo = pdev->regs.DFIFO[ch_ep_num];
    for (i = 0; i < count32b; i++, src+=4)
    {
    	a = *((uint32_t *)src);
    	USB_OTG_WRITE_REG32( fifo, a );
    }
  }
  return status;
}


/**
* @brief  USB_OTG_ReadPacket : Reads a packet from the Rx FIFO
* @param  pdev : Selected device
* @param  dest : Destination Pointer
* @param  bytes : No. of bytes
* @retval None
*/
template< typename SS, typename INTR >
void * STM32F2< SS, INTR >::USB_OTG_ReadPacket(USB_OTG_CORE_HANDLE *pdev, uint8_t *dest, uint16_t len)
{
  uint32_t i=0;
  uint32_t count32b = (len + 3) / 4;

  __IO uint32_t *fifo = pdev->regs.DFIFO[0];

  for ( i = 0; i < count32b; i++, dest += 4 )
  {
    //hzx *(__packed uint32_t *)dest = USB_OTG_READ_REG32(fifo);
	  *(uint32_t *)dest = USB_OTG_READ_REG32(fifo);
  }
  return ((void *)dest);
}

/**
* @brief  USB_OTG_GetMode : Get current mode
* @param  pdev : Selected device
* @retval current mode
*/
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::USB_OTG_GetMode(USB_OTG_CORE_HANDLE *pdev)
{
	return (USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTSTS ) & 0x1);
}


/**
* @brief  USB_OTG_HC_Halt : Halt channel
* @param  pdev : Selected device
* @param  hc_num : channel number
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_HC_Halt(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
  USB_OTG_STS status = USB_OTG_OK;
  USB_OTG_HNPTXSTS_TypeDef            nptxsts;
  USB_OTG_HPTXSTS_TypeDef             hptxsts;
  USB_OTG_HCCHAR_TypeDef              hcchar;

  nptxsts.d32 = 0;
  hptxsts.d32 = 0;
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR);
  hcchar.b.chen = 1;
  hcchar.b.chdis = 1;

  /* Check for space in the request queue to issue the halt. */
  if (hcchar.b.eptype == HCCHAR_CTRL || hcchar.b.eptype == HCCHAR_BULK)
  {
    nptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);
    if (nptxsts.b.nptxqspcavail == 0)
    {
      hcchar.b.chen = 0;
    }
  }
  else
  {
    hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);
    if (hptxsts.b.ptxqspcavail == 0)
    {
      hcchar.b.chen = 0;
    }
  }
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR, hcchar.d32);
  return status;
}


/**
* @brief  USB_OTG_ReadHostAllChannels_intr : Register PCD Callbacks
* @param  pdev : Selected device
* @retval Status
*/
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::USB_OTG_ReadHostAllChannels_intr (USB_OTG_CORE_HANDLE *pdev)
{
  return (USB_OTG_READ_REG32 (&pdev->regs.HREGS->HAINT));
}

/**
* @brief  USB_OTG_InitFSLSPClkSel : Initializes the FSLSPClkSel field of the
*         HCFG register on the PHY type
* @param  pdev : Selected device
* @param  freq : clock frequency
* @retval None
*/
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_InitFSLSPClkSel(USB_OTG_CORE_HANDLE *pdev, uint8_t freq)
{
  USB_OTG_HCFG_TypeDef   hcfg;

  hcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HCFG);
  hcfg.b.fslspclksel = freq;
  USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HCFG, hcfg.d32);
}

/**
* @brief  USB_OTG_ResetPort : Reset Host Port
* @param  pdev : Selected device
* @retval status
* @note : (1)The application must wait at least 10 ms (+ 10 ms security)
*   before clearing the reset bit.
*/
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::USB_OTG_ResetPort(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_HPRT0_TypeDef  hprt0;

	hprt0.d32 = USB_OTG_ReadHPRT0(pdev);
	hprt0.b.prtrst = 1;
	USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
	delay_ms(10);                                /* See Note #1 */
	hprt0.b.prtrst = 0;
	USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
	delay_ms(20);
	return 1;
}


/**
* @brief  Issue a ping token
* @param  None
* @retval : None
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_HC_DoPing(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
  USB_OTG_STS               status = USB_OTG_OK;
  USB_OTG_HCCHAR_TypeDef    hcchar;
  USB_OTG_HCTSIZn_TypeDef   hctsiz;

  hctsiz.d32 = 0;
  hctsiz.b.dopng = 1;
  hctsiz.b.pktcnt = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCTSIZ, hctsiz.d32);

  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR);
  hcchar.b.chen = 1;
  hcchar.b.chdis = 0;
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR, hcchar.d32);
  return status;
}

/**
* @brief  USB_OTG_ReadHPRT0 : Reads HPRT0 to modify later
* @param  pdev : Selected device
* @retval HPRT0 value
*/
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::USB_OTG_ReadHPRT0(USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HPRT0_TypeDef  hprt0;

  hprt0.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
  hprt0.b.prtena = 0;
  hprt0.b.prtconndet = 0;
  hprt0.b.prtenchng = 0;
  hprt0.b.prtovrcurrchng = 0;
  return hprt0.d32;
}

/**
  * @brief  USBH_Alloc_Channel
  *         Allocate a new channel for the pipe
  * @param  ep_addr: End point for which the channel to be allocated
  * @retval hc_num: Host channel number
  */
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::USBH_Alloc_Channel(USB_OTG_CORE_HANDLE *pdev, uint8_t ep_addr)
{
	uint16_t hc_num;

	hc_num =  USBH_GetFreeChannel(pdev);

	if (hc_num != HC_ERROR)
	{
		pdev->host.channel[hc_num] = HC_USED | ep_addr;
	}
	return hc_num;
}

/**
  * @brief  USBH_GetFreeChannel
  *         Get a free channel number for allocation to a device endpoint
  * @param  None
  * @retval idx: Free Channel number
  */
template< typename SS, typename INTR >
uint16_t STM32F2< SS, INTR >::USBH_GetFreeChannel(USB_OTG_CORE_HANDLE *pdev)
{
  uint8_t idx = 0;

  for (idx = 0 ; idx < HC_MAX ; idx++)
  {
	if ((pdev->host.channel[idx] & HC_USED) == 0)
	{
	   return idx;
	}
  }
  return HC_ERROR;
}


/**
  * @brief  USBH_Open_Channel
  *         Open a  pipe
  * @param  pdev : Selected device
  * @param  hc_num: Host channel Number
  * @param  dev_address: USB Device address allocated to attached device
  * @param  speed : USB device speed (Full/Low)
  * @param  ep_type: end point type (Bulk/int/ctl)
  * @param  mps: max pkt size
  * @retval Status
  */
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::USBH_Open_Channel  (USB_OTG_CORE_HANDLE *pdev,
                            uint8_t hc_num,
                            uint8_t dev_address,
                            uint8_t speed,
                            uint8_t ep_type,
                            uint16_t mps)
{

  pdev->host.hc[hc_num].ep_num = pdev->host.channel[hc_num]& 0x7F;
  pdev->host.hc[hc_num].ep_is_in = (pdev->host.channel[hc_num] & 0x80 ) == 0x80;
  pdev->host.hc[hc_num].dev_addr = dev_address;
  pdev->host.hc[hc_num].ep_type = ep_type;
  pdev->host.hc[hc_num].max_packet = mps;
  pdev->host.hc[hc_num].speed = speed;
  pdev->host.hc[hc_num].toggle_in = 0;
  pdev->host.hc[hc_num].isEvenTimesToggle = 0;
  pdev->host.hc[hc_num].toggle_out = 0;
  if(speed == HPRT0_PRTSPD_HIGH_SPEED)
  {
    pdev->host.hc[hc_num].do_ping = 1;
  }

  USB_OTG_HC_Init(pdev, hc_num) ;

  return HC_OK;

}


/**
  * @brief  USBH_Modify_Channel
  *         Modify a  pipe
  * @param  pdev : Selected device
  * @param  hc_num: Host channel Number
  * @param  dev_address: USB Device address allocated to attached device
  * @param  speed : USB device speed (Full/Low)
  * @param  ep_type: end point type (Bulk/int/ctl)
  * @param  mps: max pkt size
  * @retval Status
  */
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::USBH_Modify_Channel (USB_OTG_CORE_HANDLE *pdev,
                            uint8_t hc_num,
                            uint8_t dev_address,
                            uint8_t speed,
                            uint8_t ep_type,
                            uint16_t mps)
{

  if(dev_address != 0)
  {
    pdev->host.hc[hc_num].dev_addr = dev_address;
  }

  if((pdev->host.hc[hc_num].max_packet != mps) && (mps != 0))
  {
    pdev->host.hc[hc_num].max_packet = mps;
  }

  if((pdev->host.hc[hc_num].speed != speed ) && (speed != 0 ))
  {
    pdev->host.hc[hc_num].speed = speed;
  }

  USB_OTG_HC_Init(pdev, hc_num);
  return HC_OK;

}

/**
* @brief  USB_OTG_HC_Init : Prepares a host channel for transferring packets
* @param  pdev : Selected device
* @param  hc_num : channel number
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_HC_Init(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
  USB_OTG_STS status = USB_OTG_OK;
  uint32_t intr_enable = 0;
  USB_OTG_HCINTMSK_TypeDef  hcintmsk;
  USB_OTG_GINTMSK_TypeDef    gintmsk;
  USB_OTG_HCCHAR_TypeDef     hcchar;
  USB_OTG_HCINTn_TypeDef     hcint;


  gintmsk.d32 = 0;
  hcintmsk.d32 = 0;
  hcchar.d32 = 0;

  /* Clear old interrupt conditions for this host channel. */
  hcint.d32 = 0xFFFFFFFF;
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCINT, hcint.d32);

  /* Enable channel interrupts required for this transfer. */
  hcintmsk.d32 = 0;

  if (pdev->cfg.dma_enable == 1)
  {
    hcintmsk.b.ahberr = 1;
  }

  switch (pdev->host.hc[hc_num].ep_type)
  {
  case EP_TYPE_CTRL:
  case EP_TYPE_BULK:
    hcintmsk.b.xfercompl = 1;
    hcintmsk.b.stall = 1;
    hcintmsk.b.xacterr = 1;
    hcintmsk.b.datatglerr = 1;
    hcintmsk.b.nak = 1;
    if (pdev->host.hc[hc_num].ep_is_in)
    {
      hcintmsk.b.bblerr = 1;
    }
    else
    {
      hcintmsk.b.nyet = 1;
      if (pdev->host.hc[hc_num].do_ping)
      {
        hcintmsk.b.ack = 1;
      }
    }
    break;
  case EP_TYPE_INTR:
    hcintmsk.b.xfercompl = 1;
    hcintmsk.b.nak = 1;
    hcintmsk.b.stall = 1;
    hcintmsk.b.xacterr = 1;
    hcintmsk.b.datatglerr = 1;
    hcintmsk.b.frmovrun = 1;

    if (pdev->host.hc[hc_num].ep_is_in)
    {
      hcintmsk.b.bblerr = 1;
    }

    break;
  case EP_TYPE_ISOC:
    hcintmsk.b.xfercompl = 1;
    hcintmsk.b.frmovrun = 1;
    hcintmsk.b.ack = 1;

    if (pdev->host.hc[hc_num].ep_is_in)
    {
      hcintmsk.b.xacterr = 1;
      hcintmsk.b.bblerr = 1;
    }
    break;
  }


  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCINTMSK, hcintmsk.d32);


  /* Enable the top level host channel interrupt. */
  intr_enable = (1 << hc_num);
  USB_OTG_MODIFY_REG32(&pdev->regs.HREGS->HAINTMSK, 0, intr_enable);

  /* Make sure host channel interrupts are enabled. */
  gintmsk.b.hcintr = 1;
  USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, 0, gintmsk.d32);

  /* Program the HCCHAR register */
  hcchar.d32 = 0;
  hcchar.b.devaddr = pdev->host.hc[hc_num].dev_addr;
  hcchar.b.epnum   = pdev->host.hc[hc_num].ep_num;
  hcchar.b.epdir   = pdev->host.hc[hc_num].ep_is_in;
  hcchar.b.lspddev = (pdev->host.hc[hc_num].speed == HPRT0_PRTSPD_LOW_SPEED);
  hcchar.b.eptype  = pdev->host.hc[hc_num].ep_type;
  hcchar.b.mps     = pdev->host.hc[hc_num].max_packet;
  if (pdev->host.hc[hc_num].ep_type == HCCHAR_INTR)
  {
    hcchar.b.oddfrm  = 1;
  }
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR, hcchar.d32);
  return status;
}


/**
  * @brief  USBH_Free_Pipe
  *         Free the USB host channel
  * @param  idx: Channel number to be freed
  * @retval Status
  */
template< typename SS, typename INTR >
USBH_Status STM32F2< SS, INTR >::USBH_Free_Channel(USB_OTG_CORE_HANDLE *pdev, uint8_t idx)
{
   if(idx < HC_MAX)
   {
	 pdev->host.channel[idx] &= HC_USED_MASK;
   }
   return USBH_OK;
}

/**
  * @brief  USBH_CtlSendSetup
  *         Sends the Setup Packet to the Device
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer from which the Data will be send to Device
  * @param  hc_num: Host channel Number
  * @retval Status
  */
template< typename SS, typename INTR >
USBH_Status STM32F2< SS, INTR >::USBH_CtlSendSetup(USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint8_t hc_num){
  pdev->host.hc[hc_num].ep_is_in = 0;
  pdev->host.hc[hc_num].data_pid = HC_PID_SETUP;
  pdev->host.hc[hc_num].xfer_buff = buff;
  pdev->host.hc[hc_num].xfer_len = USBH_SETUP_PKT_SIZE;

  return (USBH_Status)HCD_SubmitRequest (pdev , hc_num);
}

/**
  * @brief  HCD_SubmitRequest
  *         This function prepare a HC and start a transfer
  * @param  pdev: Selected device
  * @param  hc_num: Channel number
  * @retval status
  */
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::HCD_SubmitRequest (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
	pdev->host.URB_State[hc_num] = URB_IDLE;
	pdev->host.hc[hc_num].xfer_count = 0 ;
	return USB_OTG_HC_StartXfer(pdev, hc_num);
}


/**
* @brief  USB_OTG_HC_StartXfer : Start transfer
* @param  pdev : Selected device
* @param  hc_num : channel number
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_HC_StartXfer(USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
  USB_OTG_STS status = USB_OTG_OK;
  USB_OTG_HCCHAR_TypeDef   hcchar;
  USB_OTG_HCTSIZn_TypeDef  hctsiz;
  USB_OTG_HNPTXSTS_TypeDef hnptxsts;
  USB_OTG_HPTXSTS_TypeDef  hptxsts;
  USB_OTG_GINTMSK_TypeDef  intmsk;
  uint16_t                 len_words = 0;

  uint16_t num_packets;
  uint16_t max_hc_pkt_count;

  max_hc_pkt_count = 256;
  hctsiz.d32 = 0;
  hcchar.d32 = 0;
  intmsk.d32 = 0;

  /* Compute the expected number of packets associated to the transfer */
  if (pdev->host.hc[hc_num].xfer_len > 0)
  {
    num_packets = (pdev->host.hc[hc_num].xfer_len + \
      pdev->host.hc[hc_num].max_packet - 1) / pdev->host.hc[hc_num].max_packet;
    if(!(num_packets & 0x1))
    	pdev->host.hc[hc_num].isEvenTimesToggle = 1;

    if (num_packets > max_hc_pkt_count)
    {
      num_packets = max_hc_pkt_count;
      pdev->host.hc[hc_num].xfer_len = num_packets * \
        pdev->host.hc[hc_num].max_packet;
    }
  }
  else
  {
    num_packets = 1;
  }
  if (pdev->host.hc[hc_num].ep_is_in)
  {
    pdev->host.hc[hc_num].xfer_len = num_packets * \
      pdev->host.hc[hc_num].max_packet;
  }
  /* Initialize the HCTSIZn register */
  hctsiz.b.xfersize = pdev->host.hc[hc_num].xfer_len;
  hctsiz.b.pktcnt = num_packets;
  hctsiz.b.pid = pdev->host.hc[hc_num].data_pid;
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCTSIZ, hctsiz.d32);

  if (pdev->cfg.dma_enable == 1)
  {
    USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCDMA, (unsigned int)pdev->host.hc[hc_num].xfer_buff);
  }


  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR);
  hcchar.b.oddfrm = USB_OTG_IsEvenFrame(pdev);

  /* Set host channel enable */
  hcchar.b.chen = 1;
  hcchar.b.chdis = 0;
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[hc_num]->HCCHAR, hcchar.d32);

  if (pdev->cfg.dma_enable == 0) /* Slave mode */
  {
    if((pdev->host.hc[hc_num].ep_is_in == 0) &&
       (pdev->host.hc[hc_num].xfer_len > 0))
    {
      switch(pdev->host.hc[hc_num].ep_type)
      {
        /* Non periodic transfer */
      case EP_TYPE_CTRL:
      case EP_TYPE_BULK:

        hnptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);
        len_words = (pdev->host.hc[hc_num].xfer_len + 3) / 4;

        /* check if there is enough space in FIFO space */
        if(len_words > hnptxsts.b.nptxfspcavail)
        {
          /* need to process data in nptxfempty interrupt */
          intmsk.b.nptxfempty = 1;
          USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, 0, intmsk.d32);
        }

        break;
        /* Periodic transfer */
      case EP_TYPE_INTR:
      case EP_TYPE_ISOC:
        hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);
        len_words = (pdev->host.hc[hc_num].xfer_len + 3) / 4;
        /* check if there is enough space in FIFO space */
        if(len_words > hptxsts.b.ptxfspcavail) /* split the transfer */
        {
          /* need to process data in ptxfempty interrupt */
          intmsk.b.ptxfempty = 1;
          USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, 0, intmsk.d32);
        }
        break;

      default:
        break;
      }

      /* Write packet into the Tx FIFO. */
      USB_OTG_WritePacket(pdev,
                          pdev->host.hc[hc_num].xfer_buff ,
                          hc_num, pdev->host.hc[hc_num].xfer_len);
    }
  }
  return status;
}

/**
* @brief  USB_OTG_IsEvenFrame
*         This function returns the frame number for sof packet
* @param  pdev : Selected device
* @retval Frame number
*/
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::USB_OTG_IsEvenFrame (USB_OTG_CORE_HANDLE *pdev)
{
  return !(USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM) & 0x1);
}

/**
  * @brief  HCD_GetCurrentFrame
  *         This function returns the frame number for sof packet
  * @param  pdev : Selected device
  * @retval Frame number
  *
  */
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::HCD_GetCurrentFrame (USB_OTG_CORE_HANDLE *pdev)
{
 return (USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM) & 0xFFFF) ;
}

/**
  * @brief  HCD_GetURB_State
  *         This function returns the last URBstate
  * @param  pdev: Selected device
  * @retval URB_STATE
  *
  */
template< typename SS, typename INTR >
URB_STATE STM32F2< SS, INTR >::HCD_GetURB_State (USB_OTG_CORE_HANDLE *pdev , uint8_t ch_num)
{
  return pdev->host.URB_State[ch_num] ;
}

/**
  * @brief  HCD_GetHCState
  *         This function returns the HC Status
  * @param  pdev: Selected device
  * @retval HC_STATUS
  *
  */
template< typename SS, typename INTR >
uint8_t STM32F2< SS, INTR >::HCD_GetHCState (USB_OTG_CORE_HANDLE *pdev ,  uint8_t ch_num)
{
  //return pdev->host.HC_Status[ch_num] ;
	/* not used
	 				  HC_HALTED,
	 				  HC_NYET,
	 				  HC_XACTERR,

	 *
	#define hrBADREQ    0x02
	#define hrUNDEF     0x03
	#define hrWRONGPID  0x07
	#define hrBADBC     0x08
	#define hrPIDERR    0x09
	#define hrCRCERR    0x0B
	#define hrKERR      0x0C
	#define hrJERR      0x0D
	#define hrBUSY
					  */
	HC_STATUS hcstatus_stm32 = pdev->host.HC_Status[ch_num];
	uint8_t rcode = hrSUCCESS;

	switch(hcstatus_stm32) {
		case HC_XFRC:
			rcode = hrSUCCESS;
			break;
		case HC_NAK:
			rcode = hrNAK;
			break;
		case HC_STALL:
			rcode = hrSTALL;
			break;
		case HC_BBLERR:
			rcode = hrBABBLE;
			break;
		case HC_DATATGLERR:
			rcode = hrTOGERR;
			break;
		case HC_HALTED:
		case HC_NYET:
		case HC_IDLE:	//todo: what does this mean?
			rcode = hrPKTERR;	// no matching error, so just use pkterr tempxxly
			break;
		case HC_XACTERR:
			rcode = hrTIMEOUT;
			break;
		default:
			rcode = hcstatus_stm32;
			break;
	}

	return rcode;
}

/* ------- private function members ------- */
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_BSP_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);

	/* Configure SOF VBUS ID DM DP Pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_OTG1_FS) ;
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_OTG1_FS) ;
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_OTG1_FS) ;
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG1_FS) ;

	/* this for ID line debug */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_OTG1_FS) ;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;

}

/**
  * @brief  HCD_Init
  *         Initialize the HOST portion of the driver.
  * @param  pdev: Selected device
  * @param  base_address: OTG base address
  * @retval Status
  */
template< typename SS, typename INTR >
uint32_t STM32F2< SS, INTR >::HCD_Init(USB_OTG_CORE_ID_TypeDef coreID)
{
	uint8_t i = 0;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

	pdev->host.ConnSts = 0;

	for (i= 0; i< USB_OTG_MAX_TX_FIFOS; i++)
	{
		pdev->host.ErrCnt[i]  = 0;
		pdev->host.XferCnt[i]   = 0;
		pdev->host.HC_Status[i]   = HC_IDLE;
	}
	pdev->host.hc[0].max_packet  = 8;

	USB_OTG_SelectCore(coreID);
	#ifndef DUAL_ROLE_MODE_ENABLED
	USB_OTG_DisableGlobalInt();
	USB_OTG_CoreInit();

	/* Force Host Mode*/
	USB_OTG_SetCurrentMode(HOST_MODE);
	USB_OTG_CoreInitHost();
	USB_OTG_EnableGlobalInt();
	#endif

	return 0;
}


/**
* @brief  USB_OTG_SelectCore
*         Initialize core registers address.
* @param  pdev : Selected device
* @param  coreID : USB OTG Core ID
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_SelectCore(USB_OTG_CORE_ID_TypeDef coreID)
{
	uint32_t i , baseAddress = 0;
	USB_OTG_STS status = USB_OTG_OK;

	USB_OTG_CORE_HANDLE *pdev = coreConfig;

	pdev->cfg.dma_enable       = 0;

	/* at startup the core is in FS mode */
	pdev->cfg.speed            = USB_OTG_SPEED_FULL;
	pdev->cfg.mps              = USB_OTG_FS_MAX_PACKET_SIZE ;

  /* initialize device cfg following its address */
  if (coreID == USB_OTG_FS_CORE_ID)
  {
    baseAddress                = USB_OTG_FS_BASE_ADDR;
    pdev->cfg.coreID           = USB_OTG_FS_CORE_ID;
    pdev->cfg.host_channels    = 8 ;
    pdev->cfg.dev_endpoints    = 4 ;
    pdev->cfg.TotalFifoSize    = 320; /* in 32-bits */
    pdev->cfg.phy_itface       = USB_OTG_EMBEDDED_PHY;

#ifdef USB_OTG_FS_SOF_OUTPUT_ENABLED
    pdev->cfg.Sof_output       = 1;
#endif

#ifdef USB_OTG_FS_LOW_PWR_MGMT_SUPPORT
    pdev->cfg.low_power        = 1;
#endif
  }
  else if (coreID == USB_OTG_HS_CORE_ID)
  {
    baseAddress                = USB_OTG_HS_BASE_ADDR;
    pdev->cfg.coreID           = USB_OTG_HS_CORE_ID;
    pdev->cfg.host_channels    = 12 ;
    pdev->cfg.dev_endpoints    = 6 ;
    pdev->cfg.TotalFifoSize    = 1280;/* in 32-bits */

#ifdef USB_OTG_ULPI_PHY_ENABLED
    pdev->cfg.phy_itface       = USB_OTG_ULPI_PHY;
#else
#ifdef USB_OTG_EMBEDDED_PHY_ENABLED
    pdev->cfg.phy_itface       = USB_OTG_EMBEDDED_PHY;
#endif
#endif

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
    pdev->cfg.dma_enable       = 1;
#endif

#ifdef USB_OTG_HS_SOF_OUTPUT_ENABLED
    pdev->cfg.Sof_output       = 1;
#endif

#ifdef USB_OTG_HS_LOW_PWR_MGMT_SUPPORT
    pdev->cfg.low_power        = 1;
#endif

  }

  pdev->regs.GREGS = (USB_OTG_GREGS *)(baseAddress + \
    USB_OTG_CORE_GLOBAL_REGS_OFFSET);
  pdev->regs.DREGS =  (USB_OTG_DREGS  *)  (baseAddress + \
    USB_OTG_DEV_GLOBAL_REG_OFFSET);

  for (i = 0; i < pdev->cfg.dev_endpoints; i++)
  {
    pdev->regs.INEP_REGS[i]  = (USB_OTG_INEPREGS *)  \
      (baseAddress + USB_OTG_DEV_IN_EP_REG_OFFSET + \
        (i * USB_OTG_EP_REG_OFFSET));
    pdev->regs.OUTEP_REGS[i] = (USB_OTG_OUTEPREGS *) \
      (baseAddress + USB_OTG_DEV_OUT_EP_REG_OFFSET + \
        (i * USB_OTG_EP_REG_OFFSET));
  }
  pdev->regs.HREGS = (USB_OTG_HREGS *)(baseAddress + \
    USB_OTG_HOST_GLOBAL_REG_OFFSET);
  pdev->regs.HPRT0 = (uint32_t *)(baseAddress + USB_OTG_HOST_PORT_REGS_OFFSET);

  for (i = 0; i < pdev->cfg.host_channels; i++)
  {
    pdev->regs.HC_REGS[i] = (USB_OTG_HC_REGS *)(baseAddress + \
      USB_OTG_HOST_CHAN_REGS_OFFSET + \
        (i * USB_OTG_CHAN_REGS_OFFSET));
  }
  for (i = 0; i < pdev->cfg.host_channels; i++)
  {
    pdev->regs.DFIFO[i] = (uint32_t *)(baseAddress + USB_OTG_DATA_FIFO_OFFSET +\
      (i * USB_OTG_DATA_FIFO_SIZE));
  }
  pdev->regs.PCGCCTL = (uint32_t *)(baseAddress + USB_OTG_PCGCCTL_OFFSET);

  return status;
}


/**
* @brief  USB_OTG_CoreInit
*         Initializes the USB_OTG controller registers and prepares the core
*         device mode or host mode operation.
* @param  pdev : Selected device
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_CoreInit(void)
{
	USB_OTG_STS status = USB_OTG_OK;
	USB_OTG_GUSBCFG_TypeDef  usbcfg;
	USB_OTG_GCCFG_TypeDef    gccfg;
	USB_OTG_GAHBCFG_TypeDef  ahbcfg;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

	usbcfg.d32 = 0;
	gccfg.d32 = 0;
	ahbcfg.d32 = 0;

  if (pdev->cfg.phy_itface == USB_OTG_ULPI_PHY)
  {
    gccfg.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GCCFG);
    gccfg.b.pwdn = 0;

    if (pdev->cfg.Sof_output)
    {
      gccfg.b.sofouten = 1;
    }
    USB_OTG_WRITE_REG32 (&pdev->regs.GREGS->GCCFG, gccfg.d32);

    /* Init The ULPI Interface */
    usbcfg.d32 = 0;
    usbcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GUSBCFG);

    usbcfg.b.physel            = 0; /* HS Interface */
#ifdef USB_OTG_INTERNAL_VBUS_ENABLED
    usbcfg.b.ulpi_ext_vbus_drv = 0; /* Use internal VBUS */
#else
#ifdef USB_OTG_EXTERNAL_VBUS_ENABLED
    usbcfg.b.ulpi_ext_vbus_drv = 1; /* Use external VBUS */
#endif
#endif
    usbcfg.b.term_sel_dl_pulse = 0; /* Data line pulsing using utmi_txvalid */

    usbcfg.b.ulpi_fsls = 0;
    usbcfg.b.ulpi_clk_sus_m = 0;
    USB_OTG_WRITE_REG32 (&pdev->regs.GREGS->GUSBCFG, usbcfg.d32);

    /* Reset after a PHY select  */
    USB_OTG_CoreReset();

    if(pdev->cfg.dma_enable == 1)
    {

      ahbcfg.b.hburstlen = 5; /* 64 x 32-bits*/
      ahbcfg.b.dmaenable = 1;
      USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GAHBCFG, ahbcfg.d32);

    }
  }
  else /* FS interface (embedded Phy) */
  {

    usbcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GUSBCFG);;
    usbcfg.b.physel  = 1; /* FS Interface */
    USB_OTG_WRITE_REG32 (&pdev->regs.GREGS->GUSBCFG, usbcfg.d32);
    /* Reset after a PHY select and set Host mode */
    USB_OTG_CoreReset();
    /* Deactivate the power down*/
    gccfg.d32 = 0;
    gccfg.b.pwdn = 1;

    gccfg.b.vbussensingA = 1 ;
    gccfg.b.vbussensingB = 1 ;
#ifndef VBUS_SENSING_ENABLED
    gccfg.b.disablevbussensing = 1;
#endif

    if(pdev->cfg.Sof_output)
    {
      gccfg.b.sofouten = 1;
    }

    USB_OTG_WRITE_REG32 (&pdev->regs.GREGS->GCCFG, gccfg.d32);
    delay_ms(20);
  }
  /* case the HS core is working in FS mode */
  if(pdev->cfg.dma_enable == 1)
  {

    ahbcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GAHBCFG);
    ahbcfg.b.hburstlen = 5; /* 64 x 32-bits*/
    ahbcfg.b.dmaenable = 1;
    USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GAHBCFG, ahbcfg.d32);

  }
  /* initialize OTG features */
#ifdef  USE_OTG_MODE
  usbcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GUSBCFG);
  usbcfg.b.hnpcap = 1;
  usbcfg.b.srpcap = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GUSBCFG, usbcfg.d32);
  USB_OTG_EnableCommonInt(pdev);
#endif
  return status;
}

template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_BSP_EnableInterrupt(void) {
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

/**
* @brief  USB_OTG_DisableGlobalInt
*         Enables the controller's Global Int in the AHB Config reg
* @param  pdev : Selected device
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_DisableGlobalInt(void) {
	USB_OTG_STS status = USB_OTG_OK;
	USB_OTG_GAHBCFG_TypeDef  ahbcfg;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

	ahbcfg.d32 = 0;
	ahbcfg.b.glblintrmsk = 0;	//disable interrupt	1; /* Enable interrupts */
	USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GAHBCFG, ahbcfg.d32, 0);
	return status;
}

/**
* @brief  USB_OTG_CoreReset : Soft reset of the core
* @param  pdev : Selected device
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_CoreReset(void)
{
	USB_OTG_STS status = USB_OTG_OK;
	__IO USB_OTG_GRSTCTL_TypeDef  greset;
	uint32_t count = 0;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

  greset.d32 = 0;
  /* Wait for AHB master IDLE state. */
  do
  {
    delay_us(3);
    greset.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GRSTCTL);
    if (++count > 200000)
    {
      return USB_OTG_OK;
    }
  }
  while (greset.b.ahbidle == 0);
  /* Core Soft Reset */
  count = 0;
  greset.b.csftrst = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GRSTCTL, greset.d32 );
  do
  {
    greset.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GRSTCTL);
    if (++count > 200000)
    {
      break;
    }
  }
  while (greset.b.csftrst == 1);
  /* Wait for 3 PHY Clocks*/
  delay_us(3);
  return status;
}


/**
* @brief  USB_OTG_SetCurrentMode : Set ID line
* @param  pdev : Selected device
* @param  mode :  (Host/device)
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_SetCurrentMode(uint32_t mode)
{
	USB_OTG_STS status = USB_OTG_OK;
	USB_OTG_GUSBCFG_TypeDef  usbcfg;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

	usbcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GUSBCFG);

	usbcfg.b.force_host = 0;
	usbcfg.b.force_dev = 0;

  if ( mode == HOST_MODE)
  {
    usbcfg.b.force_host = 1;
  }
  else if ( mode == DEVICE_MODE)
  {
    usbcfg.b.force_dev = 1;
  }

  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GUSBCFG, usbcfg.d32);
  delay_ms(50);
  return status;
}

/**
* @brief  USB_OTG_CoreInitHost : Initializes USB_OTG controller for host mode
* @param  pdev : Selected device
* @retval status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_CoreInitHost(void)
{
	USB_OTG_STS                     status = USB_OTG_OK;
	USB_OTG_FSIZ_TypeDef            nptxfifosize;
	USB_OTG_FSIZ_TypeDef            ptxfifosize;
	USB_OTG_HCFG_TypeDef            hcfg;
	USB_OTG_CORE_HANDLE *pdev = coreConfig;

#ifdef USE_OTG_MODE
  USB_OTG_OTGCTL_TypeDef          gotgctl;
#endif

  uint32_t                        i = 0;

  nptxfifosize.d32 = 0;
  ptxfifosize.d32 = 0;
#ifdef USE_OTG_MODE
  gotgctl.d32 = 0;
#endif
  hcfg.d32 = 0;


  /* configure charge pump IO */
  USB_OTG_BSP_ConfigVBUS();

  /* Restart the Phy Clock */
  USB_OTG_WRITE_REG32(pdev->regs.PCGCCTL, 0);

  /* Initialize Host Configuration Register */
  if (pdev->cfg.phy_itface == USB_OTG_ULPI_PHY)
  {
    USB_OTG_InitFSLSPClkSel(pdev, HCFG_30_60_MHZ);
  }
  else
  {
	  USB_OTG_InitFSLSPClkSel(pdev, HCFG_48_MHZ);
  }
  USB_OTG_ResetPort(pdev);

  hcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HCFG);
  hcfg.b.fslssupp = 0;
  USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HCFG, hcfg.d32);

  /* Configure data FIFO sizes */
  /* Rx FIFO */
#if 1 //def USB_OTG_FS_CORE
  if(pdev->cfg.coreID == USB_OTG_FS_CORE_ID)
  {
    /* set Rx FIFO size */
    USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GRXFSIZ, RX_FIFO_FS_SIZE);
    nptxfifosize.b.startaddr = RX_FIFO_FS_SIZE;
    nptxfifosize.b.depth = TXH_NP_FS_FIFOSIZ;
    USB_OTG_WRITE_REG32(&pdev->regs.GREGS->DIEPTXF0_HNPTXFSIZ, nptxfifosize.d32);

    ptxfifosize.b.startaddr = RX_FIFO_FS_SIZE + TXH_NP_FS_FIFOSIZ;
    ptxfifosize.b.depth     = TXH_P_FS_FIFOSIZ;
    USB_OTG_WRITE_REG32(&pdev->regs.GREGS->HPTXFSIZ, ptxfifosize.d32);
  }
#endif

  /* Make sure the FIFOs are flushed. */
  USB_OTG_FlushTxFifo(0x10);         /* all Tx FIFOs */
  USB_OTG_FlushRxFifo();


  /* Clear all pending HC Interrupts */
  for (i = 0; i < pdev->cfg.host_channels; i++)
  {
    USB_OTG_WRITE_REG32( &pdev->regs.HC_REGS[i]->HCINT, 0xFFFFFFFF );
    USB_OTG_WRITE_REG32( &pdev->regs.HC_REGS[i]->HCINTMSK, 0 );
  }
#ifndef USE_OTG_MODE
  USB_OTG_DriveVbus(1);
#endif

  USB_OTG_EnableHostInt();
  return status;
}

/**
* @brief  USB_OTG_FlushTxFifo : Flush a Tx FIFO
* @param  pdev : Selected device
* @param  num : FO num
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_FlushTxFifo (uint32_t num)
{
  USB_OTG_STS status = USB_OTG_OK;
  __IO USB_OTG_GRSTCTL_TypeDef  greset;
  USB_OTG_CORE_HANDLE *pdev = coreConfig;

  uint32_t count = 0;
  greset.d32 = 0;
  greset.b.txfflsh = 1;
  greset.b.txfnum  = num;
  USB_OTG_WRITE_REG32( &pdev->regs.GREGS->GRSTCTL, greset.d32 );
  do
  {
    greset.d32 = USB_OTG_READ_REG32( &pdev->regs.GREGS->GRSTCTL);
    if (++count > 200000)
    {
      break;
    }
  }
  while (greset.b.txfflsh == 1);
  /* Wait for 3 PHY Clocks*/
  delay_us(3);
  return status;
}


/**
* @brief  USB_OTG_FlushRxFifo : Flush a Rx FIFO
* @param  pdev : Selected device
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_FlushRxFifo(void)
{
  USB_OTG_STS status = USB_OTG_OK;
  __IO USB_OTG_GRSTCTL_TypeDef  greset;
  uint32_t count = 0;
  USB_OTG_CORE_HANDLE *pdev = coreConfig;

  greset.d32 = 0;
  greset.b.rxfflsh = 1;
  USB_OTG_WRITE_REG32( &pdev->regs.GREGS->GRSTCTL, greset.d32 );
  do
  {
    greset.d32 = USB_OTG_READ_REG32( &pdev->regs.GREGS->GRSTCTL);
    if (++count > 200000)
    {
      break;
    }
  }
  while (greset.b.rxfflsh == 1);
  /* Wait for 3 PHY Clocks*/
  delay_us(3);
  return status;
}


/**
* @brief  USB_OTG_DriveVbus : set/reset vbus
* @param  pdev : Selected device
* @param  state : VBUS state
* @retval None
*/
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_DriveVbus(uint8_t state)
{
  USB_OTG_HPRT0_TypeDef     hprt0;
  USB_OTG_CORE_HANDLE *pdev = coreConfig;

  hprt0.d32 = 0;

  /* enable disable the external charge pump */
  USB_OTG_BSP_DriveVBUS(state);

  /* Turn on the Host port power. */
  hprt0.d32 = USB_OTG_ReadHPRT0(pdev);
  if ((hprt0.b.prtpwr == 0 ) && (state == 1 ))
  {
    hprt0.b.prtpwr = 1;
    USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
  }
  if ((hprt0.b.prtpwr == 1 ) && (state == 0 ))
  {
    hprt0.b.prtpwr = 0;
    USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0.d32);
  }

  delay_ms(200);
}

/**
* @brief  USB_OTG_EnableHostInt: Enables the Host mode interrupts
* @param  pdev : Selected device
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_EnableHostInt(void)
{
  USB_OTG_STS       status = USB_OTG_OK;
  USB_OTG_GINTMSK_TypeDef  intmsk;
  intmsk.d32 = 0;
  USB_OTG_CORE_HANDLE *pdev = coreConfig;

  /* Disable all interrupts. */
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTMSK, 0);

  /* Clear any pending interrupts. */
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, 0xFFFFFFFF);

  /* Enable the common interrupts */
  USB_OTG_EnableCommonInt();

  if (pdev->cfg.dma_enable == 0)
  {
    intmsk.b.rxstsqlvl  = 1;
  }
  intmsk.b.portintr   = 1;
  intmsk.b.hcintr     = 1;
  intmsk.b.disconnect = 1;
  intmsk.b.sofintr    = 1;
  intmsk.b.incomplisoout  = 1;
  USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, intmsk.d32);
  return status;
}

/**
* @brief  USB_OTG_EnableCommonInt
*         Initializes the commmon interrupts, used in both device and modes
* @param  pdev : Selected device
* @retval None
*/
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_EnableCommonInt(void)
{
  USB_OTG_GINTMSK_TypeDef  int_mask;
  USB_OTG_CORE_HANDLE *pdev = coreConfig;

  int_mask.d32 = 0;
  /* Clear any pending USB_OTG Interrupts */
#ifndef USE_OTG_MODE
  USB_OTG_WRITE_REG32( &pdev->regs.GREGS->GOTGINT, 0xFFFFFFFF);
#endif
  /* Clear any pending interrupts */
  USB_OTG_WRITE_REG32( &pdev->regs.GREGS->GINTSTS, 0xBFFFFFFF);
  /* Enable the interrupts in the INTMSK */
  int_mask.b.wkupintr = 1;
  int_mask.b.usbsuspend = 1;

#ifdef USE_OTG_MODE
  int_mask.b.otgintr = 1;
  int_mask.b.sessreqintr = 1;
  int_mask.b.conidstschng = 1;
#endif
  USB_OTG_WRITE_REG32( &pdev->regs.GREGS->GINTMSK, int_mask.d32);
}

/**
* @brief  USB_OTG_EnableGlobalInt
*         Enables the controller's Global Int in the AHB Config reg
* @param  pdev : Selected device
* @retval USB_OTG_STS : status
*/
template< typename SS, typename INTR >
USB_OTG_STS STM32F2< SS, INTR >::USB_OTG_EnableGlobalInt(void)
{
  USB_OTG_STS status = USB_OTG_OK;
  USB_OTG_GAHBCFG_TypeDef  ahbcfg;
  USB_OTG_CORE_HANDLE *pdev = coreConfig;

  ahbcfg.d32 = 0;
  ahbcfg.b.glblintrmsk = 1; /* Enable interrupts */
  USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GAHBCFG, 0, ahbcfg.d32);
  return status;
}

/* ------- static members ------- */
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_BSP_ConfigVBUS(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOH , ENABLE);

	GPIO_InitStructure.GPIO_Pin = HOST_POWERSW_VBUS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(HOST_POWERSW_PORT,&GPIO_InitStructure);
	/* By Default, DISABLE is needed on output of the Power Switch */
	GPIO_SetBits(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);

	delay_ms(200);   /* Delay is need for stabilising the Vbus Low
	in Reset Condition, when Vbus=1 and Reset-button is pressed by user */
}


/**
  * @brief  BSP_Drive_VBUS
  *         Drives the Vbus signal through IO
  * @param  state : VBUS states
  * @retval None
  */
template< typename SS, typename INTR >
void STM32F2< SS, INTR >::USB_OTG_BSP_DriveVBUS(uint8_t state)
{
  /*
  On-chip 5 V VBUS generation is not supported. For this reason, a charge pump
  or, if 5 V are available on the application board, a basic power switch, must
  be added externally to drive the 5 V VBUS line. The external charge pump can
  be driven by any GPIO output. When the application decides to power on VBUS
  using the chosen GPIO, it must also set the port power bit in the host port
  control and status register (PPWR bit in OTG_FS_HPRT).

  Bit 12 PPWR: Port power
  The application uses this field to control power to this port, and the core
  clears this bit on an overcurrent condition.
  */
#ifndef USE_USB_OTG_HS
  if (0 == state)
  {
    /* DISABLE is needed on output of the Power Switch */
    GPIO_SetBits(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
  }
  else
  {
    /*ENABLE the Power Switch by driving the Enable LOW */
    GPIO_ResetBits(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
  }
#endif
}
#endif //_USBHOST_H_
