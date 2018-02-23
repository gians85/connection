/* mbed Microcontroller Library
 */
#include <stddef.h>
#include "us_ticker_api.h"
#include "PeripheralNames.h"
#include "hal_tick.h"

#include "BlueNRG1_sysCtrl.h"
#include "BlueNRG1_mft.h"
#include "bluenrg_x_device.h"

//#include "serial_mylib.h"


uint32_t carry=0, b=0, c=0;

void MFT1B_IRQHandler(void){
    if ( MFT_StatusIT(MFT1,MFT_IT_TND) != RESET ){
    	carry = carry + 0x10000;
        // Clear MFT2 pending interrupt A
        MFT_ClearIT(MFT1, MFT_IT_TND);
	}
}


void us_ticker_init(void){
	NVIC_InitType NVIC_InitStructure;
	MFT_InitType timer_init;

	carry = 0;

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1 | CLOCK_PERIPH_MTFX2, ENABLE);
	MFT_StructInit(&timer_init);

	timer_init.MFT_Mode = MFT_MODE_3;
	timer_init.MFT_Prescaler = 15;

	/* MFT1 configuration */
	timer_init.MFT_Clock1 = MFT_NO_CLK;
	timer_init.MFT_Clock2 = MFT_PRESCALED_CLK;
	timer_init.MFT_CRB = 0xFFFF;
	MFT_Init(MFT1, &timer_init);

	/* Set counter for timer2 */
	MFT_SetCounter2(MFT1, 0xFFFF);

	/* Enable MFT1B Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = MFT1B_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIGH_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the MFT1 interrupt */
	MFT_EnableIT(MFT1, MFT_IT_TND, ENABLE);
}


uint32_t us_ticker_read(){
	b = MFT_GetCounter2(MFT1);
	return (carry + (0x0000FFFF-b));
}

void us_ticker_set_interrupt(timestamp_t timestamp){
	/* Enable the MFT1 */
	MFT_Cmd(MFT1, ENABLE);
}

/* NOTE: must be called with interrupts disabled! */
void us_ticker_disable_interrupt(void){
	/* Disable the MFT1 */
	MFT_Cmd(MFT1, DISABLE);
}

/* NOTE: must be called with interrupts disabled! */
void us_ticker_clear_interrupt(void){
}

/* NOTE: must be called with interrupts disabled! */
void us_ticker_fire_interrupt(void){
}



#ifdef fff
uint32_t a;
void us_ticker_init(void){
}

uint32_t us_ticker_read(){
	return a;
}

void us_ticker_set_interrupt(timestamp_t timestamp){
}

void us_ticker_disable_interrupt(void){
}

void us_ticker_clear_interrupt(void){
}

void us_ticker_fire_interrupt(void){
}
#endif
