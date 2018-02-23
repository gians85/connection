/**
  ******************************************************************************
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m0
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata

/**
 * This is the code that gets called when the processor first
 * starts execution following a reset event. Only the absolutely
 * necessary set is performed, after which the application
 * supplied main() routine is called.
*/
  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function

Reset_Handler:
/* set stack pointer */
  ldr   r0, =_estack
  mov   sp, r0


/* Copy the data segment initializers from flash to SRAM
  movs r1, #0
  b LoopCopyDataInit
  
CopyDataInit:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4
  
LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1     //r2=r0 & condition code flags are updated
  cmp r2, r3
  bcc CopyDataInit    //branch if Carry=0 : b if r2<r3
*/


/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3     //r4=r0 & condition code flags are updated
  cmp r4, r1          //branch if Carry=0 : b if r4<r1
  bcc CopyDataInit


/* Call the clock system intitialization function */
    bl  SystemInit

/* Call the application's entry point */
    bl _start


LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  0xAA5555AA
  .word  0x424C5545
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  0
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  .word  GPIO_Handler                   	/* GPIO			            	*/
  .word  NVM_Handler                  	/* FLASH                   	 	*/
  .word  0
  .word  0
  .word	 UART_Handler                   	/* UART			            	*/
  .word  SPI_Handler                   	/* SPI			            	*/
  .word  Blue_Handler                   	/* BLE controller			*/
  .word  WDG_Handler                   	/* WatchDog        		(WWDG)	*/
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  ADC_Handler              		/* ADC                          */
  .word  I2C1_Handler              		/* I2C1                         */
  .word  I2C2_Handler              		/* I2C2                         */
  .word  0
  .word  MFT1A_Handler              		/* MFT1 A                       */
  .word  MFT1B_Handler              		/* MFT1 B                       */
  .word  MFT2A_Handler              		/* MFT2 A                       */
  .word  MFT2B_Handler              		/* MFT2 B                       */
  .word  RTC_Handler              		/* RTC                          */
  .word  PKA_Handler              		/* PKA                          */
  .word  DMA_Handler              		    /* DMA                          */
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak      GPIO_Handler
  .thumb_set GPIO_Handler,Default_Handler

  .weak      NVM_Handler
  .thumb_set NVM_Handler,Default_Handler

  .weak      UART_Handler
  .thumb_set UART_Handler,Default_Handler

  .weak      SPI_Handler
  .thumb_set SPI_Handler,Default_Handler

  .weak      Blue_Handler
  .thumb_set Blue_Handler,Default_Handler

  .weak      WDG_Handler
  .thumb_set WDG_Handler,Default_Handler

  .weak      ADC_Handler
  .thumb_set ADC_Handler,Default_Handler

  .weak      I2C1_Handler
  .thumb_set I2C1_Handler,Default_Handler

  .weak      I2C2_Handler
  .thumb_set I2C2_Handler,Default_Handler

  .weak      MFT1A_Handler
  .thumb_set MFT1A_Handler,Default_Handler

  .weak      MFT1B_Handler
  .thumb_set MFT1B_Handler,Default_Handler

  .weak      MFT2A_Handler
  .thumb_set MFT2A_IRQHandler,Default_Handler

  .weak      MFT2B_Handler
  .thumb_set MFT2B_Handler,Default_Handler

  .weak      RTC_Handler
  .thumb_set RTC_Handler,Default_Handler

  .weak      PKA_Handler
  .thumb_set PKA_Handler,Default_Handler

  .weak      DMA_Handler
  .thumb_set DMA_Handler,Default_Handler


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

