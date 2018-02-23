/* mbed Microcontroller Library
 */

#ifndef MBED_MBED_RTX_H
#define MBED_MBED_RTX_H

#ifndef INITIAL_SP

#if (defined(TARGET_BLUENRG1))
#define INITIAL_SP              (0x20006000UL)

#elif (defined(TARGET__BLUENRG2))
#define INITIAL_SP              (0x00000000UL)  //TBC


#else
#error "INITIAL_SP is not defined for this target in the mbed_rtx.h file"
#endif

#endif // INITIAL_SP

#endif  // MBED_MBED_RTX_H
