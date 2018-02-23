/*
 * pin_device.h
 *
 *  Created on: 29 nov 2017
 *      Author: Gianluca
 */

#ifndef MBED_OS_TARGETS_TARGET_STMBLUE_PIN_DEVICE_H_
#define MBED_OS_TARGETS_TARGET_STMBLUE_PIN_DEVICE_H_

//#include "BlueNRG_x_device.h"
#include "BlueNRG1_gpio.h"
#include "BlueNRG1_sysCtrl.h"
#include "PinNames.h"


#define LED_ON                          Bit_SET
#define LED_OFF                         Bit_RESET


uint32_t getGpioPin(PinName);


#endif /* MBED_OS_TARGETS_TARGET_STMBLUE_PIN_DEVICE_H_ */
