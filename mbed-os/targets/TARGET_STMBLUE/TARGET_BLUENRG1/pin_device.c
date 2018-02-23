/*
 * pin_device.c
 *
 *  Created on: 29 nov 2017
 *      Author: Gianluca
 */


#include "pin_device.h"


uint32_t getGpioPin(PinName pin){
	return (uint32_t)(1 << ((uint32_t)pin & 0xF)); // Return the pin mask
	/*uint32_t mask;
	switch (pin)
	{
		case(IO_0):
			mask = GPIO_Pin_0;
			break;
		case(IO_1):
			mask = GPIO_Pin_1;
			break;
		case(IO_2):
			mask = GPIO_Pin_2;
			break;
		case(IO_3):
			mask = GPIO_Pin_3;
			break;
		case(IO_4):
			mask = GPIO_Pin_4;
			break;
		case(IO_5):
			mask = GPIO_Pin_5;
			break;
		case(IO_6):
			mask = GPIO_Pin_6;
			break;
		case(IO_7):
			mask = GPIO_Pin_7;
			break;
		case(IO_8):
			mask = GPIO_Pin_8;
			break;
		case(IO_9):
			mask = GPIO_Pin_9;
			break;
		case(IO_10):
			mask = GPIO_Pin_10;
			break;
		case(IO_11):
			mask = GPIO_Pin_11;
			break;
		case(IO_12):
			mask = GPIO_Pin_12;
			break;
		case(IO_13):
			mask = GPIO_Pin_13;
			break;
		case(IO_14):
			mask = GPIO_Pin_14;
			break;
		case(IO_15):
			mask = GPIO_Pin_15;
			break;
		default:
			mask = GPIO_Pin_14;  //LED3
			break;
	}
	return (mask);*/
}


