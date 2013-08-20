
#ifndef _bsp_h_
#define _bsp_h_
#ifdef __cplusplus
 extern "C" {
#endif

/* CMSIS includes. */
#include "stm32f2xx_usart.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"
#include "usb_hcd_int.h"

#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */


#include "stm32f2xx.h"
#include "stm322xg_eval.h"

void BSP_init(void);
uint32_t millis(void);
__inline void delay_ms(uint32_t count);
__inline void delay_us(uint32_t count);
#define delay delay_ms
extern USB_OTG_CORE_HANDLE USB_OTG_Core_dev;

void __cxa_pure_virtual(void);
//#define __init      __attribute__ ((__section__ (".init.text")))
#ifdef __cplusplus
}
#endif
#endif
