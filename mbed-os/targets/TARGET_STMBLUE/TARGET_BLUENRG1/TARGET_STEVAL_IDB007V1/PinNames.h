/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright
 *******************************************************************************
 */
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IO_0  = 0x00,
	IO_1  = 0x01,
	IO_2  = 0x02,
	IO_3  = 0x03,
	IO_4  = 0x04,
	IO_5  = 0x05,
	IO_6  = 0x06,
	IO_7  = 0x07,
	IO_8  = 0x08,
	IO_9  = 0x09,
	IO_10 = 0x0A,
	IO_11 = 0x0B,
	IO_12 = 0x0C,
	IO_13 = 0x0D,
	IO_14 = 0x0E,
	IO_15 = 0x0F,


    // ADC internal channels
	ADC_OPEN = 0x00,
	ADC2     = 0x01,
	ADC1     = 0x02,
	ADC_DIFF = 0x03,
    ADC_TEMP = 0x04,
    ADC_VBAT = 0x05,
	ADC_VREF = 0x06,


    // STEVAL_IDB007V1 signals namings
	// DIO
	DIO0 = IO_0,
	DIO1 = IO_1,
	DIO2 = IO_2,
	DIO3 = IO_3,
	DIO4 = IO_4,
	DIO5 = IO_5,
	DIO6 = IO_6,
	DIO7 = IO_7,
	DIO8 = IO_8,
	DIO11 = IO_11,
	DIO12 = IO_12,
	DIO13 = IO_13,
	DIO14 = IO_14,
	// LEDS
	DL1 = DIO6,
	DL2 = DIO7,
	DL3 = DIO14,
	// PUSH BUTTONS
	PUSH1 = DIO13,
	PUSH2 = DIO5,


	// Arduino connector namings
	A0          = DIO12,
	//A1          = TEST1,
	A2          = DIO13,
	A3          = DIO14,
	A4          = ADC1,
	A5          = ADC2,
	D0          = DIO11,
	D1          = DIO8,
	D2          = DIO11,
	D3          = DIO0,
	D4          = DIO2,
	D5          = DIO3,
	D6          = DIO6,
	//D7          = RESET,
	D8          = DIO8,
	D9          = DIO7,
	D10         = DIO1,
	D11         = DIO2,
	D12         = DIO3,
	D13         = DIO0,
	D14         = DIO5,
	D15         = DIO4,


    // Generic signals namings
    LED1           = DL1,
    LED2           = DL2,
    LED3           = DL3,
	USER_BUTTON    = PUSH1,
    USER_BUTTON_1  = PUSH1,
	USER_BUTTON_2  = PUSH2,

    // Standardized button names
	BUTTON      = USER_BUTTON_1,
    BUTTON1     = USER_BUTTON_1,
	BUTTON2     = USER_BUTTON_2,
    SERIAL_TX   = DIO8,
    SERIAL_RX   = DIO11,
    I2C_SCL     = DIO5,
    I2C_SDA     = DIO4,
    SPI_MOSI    = DIO2,
    SPI_MISO    = DIO3,
    SPI_SCK     = DIO0,
    SPI_CS      = DIO1,
	USBTX		= SERIAL_TX,
	USBRX		= SERIAL_RX,

    // Not connected
    NC = (int)0xFFFFFFFF
} PinName;

typedef enum {
	PullNone = 0,
	PullEnable = 1,
	PullDefault = PullNone,
    PullUp = PullEnable,
    PullDown = PullNone,
	NoPull = PullNone
} PinMode;

typedef enum {
    PIN_INPUT = 0,
    PIN_OUTPUT
} PinDirection;

typedef enum {
	PortA = 0
} PortName;

/*typedef enum {
    PullNone          = 0,
    PullUp            = 1,
    PullDown          = 2,
    OpenDrainPullUp   = 3,
    OpenDrainNoPull   = 4,
    OpenDrainPullDown = 5,
    PushPullNoPull    = PullNone,
    PushPullPullUp    = PullUp,
    PushPullPullDown  = PullDown,
    OpenDrain         = OpenDrainPullUp,
    PullDefault       = PullNone
} PinMode;*/


#ifdef __cplusplus
}
#endif

#endif
