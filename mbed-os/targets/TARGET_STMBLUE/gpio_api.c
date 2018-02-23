/* mbed Microcontroller Library
 *******************************************************************************
 *******************************************************************************
 */
#include "mbed_assert.h"	// arm
#include "gpio_api.h"		// arm
#include "pinmap.h"			// arm
#include "mbed_error.h"		// arm
#include "pin_device.h"

//#include "serial_mylib.h"

//extern const uint32_t ll_pin_defines[16];

#define GPIOA_BASE GPIO_BASE

// Enable GPIO clock and return GPIO base address
GPIO_TypeDef *Set_GPIO_Clock(uint32_t port_idx) {
    uint32_t gpio_add = 0;
    switch (port_idx) {
        case PortA:
            gpio_add = GPIOA_BASE;
            //__HAL_RCC_GPIOA_CLK_ENABLE();
            SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
            break;
        default:
            error("Pinmap error: wrong port number.");
            break;
    }
    return (GPIO_TypeDef *) gpio_add;
}


void gpio_mode(gpio_t *obj, PinMode mode) {
	uint8_t current_mode = obj->GPIO_Mode;
	if (mode){
		obj->GPIO_Pull = ENABLE;
	}
	else{
		obj->GPIO_Pull = DISABLE;
	}
	if (obj->GPIO_Mode==GPIO_Output) {
		obj->GPIO_Pull = ENABLE;
		obj->GPIO_HighPwr = ENABLE;
	}
	GPIO_Init(obj);
	obj->GPIO_Mode = current_mode;
}

inline void gpio_dir(gpio_t *obj, PinDirection direction) {
	uint8_t current_mode;
	if (direction){
		obj->GPIO_Mode = GPIO_Output;
		current_mode = GPIO_Output;
		obj->GPIO_Pull = ENABLE;
		obj->GPIO_HighPwr = ENABLE;
	}
	else{
		obj->GPIO_Mode = GPIO_Input;
		current_mode = GPIO_Input;
		obj->GPIO_Pull = DISABLE;
		obj->GPIO_HighPwr = DISABLE;
	}
	GPIO_Init(obj);
	obj->GPIO_Mode = current_mode;
}

inline void gpio_write(gpio_t *obj, int value){
	if (value!=0){
		GPIO_WriteBit(obj->GPIO_Pin, Bit_SET);
	}
    else{
    	GPIO_WriteBit(obj->GPIO_Pin, Bit_RESET);
    }
}


uint32_t gpio_set(PinName pin) {
	MBED_ASSERT(pin != (PinName)NC);
	// pin_function(): pinmap.h:ARM
	//void pin_function(PinName pin, int function);
	//pin_function(pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
	//return (uint32_t)(1 << ((uint32_t)pin & 0xF)); // Return the pin mask
	int data = (int)GPIO_Input;
	pin_function(pin, data);
	return getGpioPin(pin);
}



void gpio_init(gpio_t *obj, PinName pin) {
	/* Enable the GPIO Clock */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
	// get pin mask
	obj->GPIO_Pin = getGpioPin(pin);
	// preset
	obj->GPIO_Mode = GPIO_Output;
	obj->GPIO_Pull = ENABLE;
	obj->GPIO_HighPwr = ENABLE;
	GPIO_Init(obj);
	obj->GPIO_Mode = GPIO_Output;
}

int gpio_read(gpio_t *obj){
	if (GPIO_ReadBit(obj->GPIO_Pin))
	    return SET;
	  else
	    return RESET;
}





#ifdef stm

uint32_t gpio_set(PinName pin) {
	    MBED_ASSERT(pin != (PinName)NC);
	    //pin_function(): pinmap.h:ARM -> pinmap.c:STM
	    pin_function(pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
	    return (uint32_t)(1 << ((uint32_t)pin & 0xF)); // Return the pin mask
}

void gpio_init(gpio_t *obj, PinName pin) {
    obj->pin = pin;
    if (pin == (PinName)NC) {
        return;
    }
    uint32_t port_index = STM_PORT(pin);
    // Enable GPIO clock
    GPIO_TypeDef *gpio = Set_GPIO_Clock(port_index);
    // Fill GPIO object structure for future use
    obj->mask    = gpio_set(pin);
    obj->gpio  = gpio;
    obj->ll_pin  = ll_pin_defines[STM_PIN(obj->pin)];
    obj->reg_in  = &gpio->IDR;
    obj->reg_set = &gpio->BSRR;
#ifdef GPIO_IP_WITHOUT_BRR
    obj->reg_clr = &gpio->BSRR;
#else
    obj->reg_clr = &gpio->BRR;
#endif
}

void gpio_mode(gpio_t *obj, PinMode mode) {
    pin_mode(obj->pin, mode);
}

inline void gpio_dir(gpio_t *obj, PinDirection direction) {
    if (direction == PIN_INPUT) {
        LL_GPIO_SetPinMode(obj->gpio, obj->ll_pin, LL_GPIO_MODE_INPUT);
    } else {
        LL_GPIO_SetPinMode(obj->gpio, obj->ll_pin, LL_GPIO_MODE_OUTPUT);
    }
}
#endif
