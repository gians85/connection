;*******************************************************************************
;*
;* File Name          : startup_stm32f070xb.s
;*
;*******************************************************************************
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler                  ; Reset Handler

        DCD     NMI_Handler                    ; NMI Handler
        DCD     HardFault_Handler              ; Hard Fault Handler
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     SVC_Handler                    ; SVCall Handler
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     PendSV_Handler                 ; PendSV Handler
        DCD     SysTick_Handler                ; SysTick Handler

        ; External Interrupts
        DCD     GPIO_Handler                ; Reserved
        DCD     NVM_Handler               ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     UART_Handler                ; Reserved
        DCD     SPI_Handler                 ; Reserved
        DCD     Blue_Handler                   ; Reserved
        DCD     WDG_Handler                 ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     ADC_Handler                 ; Reserved
        DCD     I2C1_Handler                ; Reserved
        DCD     I2C2_Handler                ; Reserved
        DCD     0                              ; Reserved
        DCD     MFT1A_Handler               ; Reserved
        DCD     MFT1B_Handler               ; Reserved
        DCD     MFT2A_Handler               ; Reserved
        DCD     MFT2B_Handler               ; Reserved
        DCD     RTC_Handler                 ; Reserved
        DCD     PKA_Handler                    ; Reserved
        DCD     DMA_Handler                    ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK GPIO_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
GPIO_Handler
        B GPIO_Handler

        PUBWEAK NVM_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NVM_Handler
        B NVM_Handler

        PUBWEAK UART_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART_Handler
        B UART_Handler

        PUBWEAK SPI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI_Handler
        B SPI_Handler

        PUBWEAK Blue_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
Blue_Handler
        B Blue_Handler

        PUBWEAK WDG_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
WDG_Handler
        B WDG_Handler

        PUBWEAK ADC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
ADC_Handler
        B ADC_Handler

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C1_Handler
        B I2C1_Handler

        PUBWEAK I2C2_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C2_Handler
        B I2C2_Handler

        PUBWEAK MFT1A_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT1A_Handler
        B MFT1A_Handler

        PUBWEAK MFT1B_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT1B_Handler
        B MFT1B_Handler

        PUBWEAK MFT2A_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT2A_Handler
        B MFT2A_Handler

        PUBWEAK MFT2B_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT2B_Handler
        B MFT2B_Handler

        PUBWEAK RTC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
RTC_Handler
        B RTC_Handler

        PUBWEAK PKA_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PKA_Handler
        B PKA_Handler

        PUBWEAK DMA_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Handler
        B DMA_Handler

        END
;*****************************END OF FILE*****
