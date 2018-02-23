/* mbed Microcontroller Library
 * CMSIS-style functionality to support dynamic vectors
 *******************************************************************************
 *******************************************************************************
 */

#ifndef MBED_CMSIS_NVIC_H
#define MBED_CMSIS_NVIC_H

// CORE: 16 vectors = 64 bytes from 0x00 to 0x3F
// MCU Peripherals: 32 vectors = 128 bytes from 0x40 to 0xBF
// Total: 48 vectors = 192 bytes (0xC0) to be reserved in RAM
#define NVIC_NUM_VECTORS      48
#define NVIC_USER_IRQ_OFFSET  16

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NVIC_RAM_VECTOR_ADDRESS		_MEMORY_RAM_BEGIN_		// Vectors positioned at start of RAM
#define NVIC_FLASH_VECTOR_ADDRESS	_MEMORY_FLASH_BEGIN_	// Initial vector position in flash

void NVIC_SetVector(IRQn_Type IRQn, uint32_t vector);
uint32_t NVIC_GetVector(IRQn_Type IRQn);

#include "misc.h"

#ifdef __cplusplus
}
#endif

#endif
