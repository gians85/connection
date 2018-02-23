/*
 * imu.h
 *
 *  Created on: 06 feb 2018
 *      Author: Gianluca
 */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "mbed.h"

#define READ        0xFF
#define READ_1B     0x80
#define WHO_AM_I    0x0F
#define I_AM_XG     0b01101001

#define OUT_X_L_A   0x28
#define OUT_X_H_A   0x29
#define OUT_Y_L_A   0x2A
#define OUT_Y_H_A   0x2B
#define OUT_Z_L_A   0x2C
#define OUT_Z_H_A   0x2D

#define OUT_X_L_G   0x22
#define OUT_X_H_G   0x23
#define OUT_Y_L_G   0x24
#define OUT_Y_H_G   0x25
#define OUT_Z_L_G   0x26
#define OUT_Z_H_G   0x27

#define OUT_TEMP_L  0x20
#define OUT_TEMP_H  0x21


#endif /* IMU_IMU_H_ */
