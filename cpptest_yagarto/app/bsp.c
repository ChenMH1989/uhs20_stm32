
#include "bsp.h"

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __uart_putchar(int ch)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(EVAL_COM1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
	{}

	return ch;
}

void BSP_init(void) {
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	STM_EVAL_COMInit(COM1, &USART_InitStructure);

	printf("\n\n\rUSART Printf Example: retarget the C library printf function to the USART\n");


}
static uint32_t tick_time = 0;
uint32_t millis(void) {
	return tick_time;
}
__inline void delay_ms(uint32_t count) {
	//TODO: if the systick is not enabled, we should implement this
	//delay function with a while(cnt){cnt--;nop;} loop.
	uint32_t end_time = millis() + (count >> 1);
	while(end_time - millis());
}

__inline void delay_us(uint32_t count) {
	count = count * (SystemCoreClock / 1000000);
	for(; count > 0; count--);
}

void SysTick_Handler(void) {
	tick_time++;
}

void Default_Handler_c(unsigned int * hardfault_args) {
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);

	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	printf ("\n\n[Hard fault handler - all numbers in hex]\n");
	printf ("R0 = %x\n", stacked_r0);
	printf ("R1 = %x\n", stacked_r1);
	printf ("R2 = %x\n", stacked_r2);
	printf ("R3 = %x\n", stacked_r3);
	printf ("R12 = %x\n", stacked_r12);
	printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
	printf ("PC [R15] = %x  program counter\n", stacked_pc);
	printf ("PSR = %x\n", stacked_psr);
	printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
	printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
	printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
	printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
	printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
	printf ("SCB_SHCSR = %x\n", SCB->SHCSR);

	while (1);
}
void OTG_FS_IRQHandler(void)
{
	USBH_OTG_ISR_Handler(&USB_OTG_Core_dev);
	//while(1);
}

void __cxa_pure_virtual(void) { while (1); }
