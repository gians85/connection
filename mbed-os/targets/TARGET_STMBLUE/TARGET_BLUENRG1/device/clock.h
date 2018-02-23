/**
  ******************************************************************************
  ******************************************************************************
  */
#ifndef __CLOCK_H__
#define __CLOCK_H__

#include "BlueNRG_x_device.h"

/** 
 * @brief Number of clocks in one seconds.
 */
extern const uint32_t CLOCK_SECOND;

typedef uint32_t tClockTime;

/** 
 * @brief This function initializes the clock library based on SysTick.
 * 
 * @param None
 *
 * @retval None
 */
void Clock_Init(void);

/** 
 * @brief This function returns the current system clock time.
 * 
 * @param None
 *
 * @retval It returns current clock time, measured in system ticks.
 */
tClockTime Clock_Time(void);


/** 
 * @brief This function waits for a given number of milliseconds.
 * 
 * @param i delay in milliseconds
 *
 * @retval None
 */
void Clock_Wait(uint32_t i);

/**
 * @brief This function is called on SysTick_Handler()
 *
 */
void SysCount_Handler(void);

#endif /* __CLOCK_H__ */

