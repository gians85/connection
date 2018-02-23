#include "cmsis_nvic.h"

#ifdef stm32
void NVIC_SetVector(IRQn_Type IRQn, uint32_t vector) {
    int i;
    /*
    // Copy and switch to dynamic vectors if first time called
    if ((SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE) != SYSCFG_CFGR1_MEM_MODE) {
        uint32_t *old_vectors = (uint32_t *)NVIC_FLASH_VECTOR_ADDRESS;
        for (i = 0; i < NVIC_NUM_VECTORS; i++) {
            *((uint32_t *)(NVIC_RAM_VECTOR_ADDRESS + (i*4))) = old_vectors[i];
        }
        SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE; // Embedded SRAM mapped at 0x00000000
    }*/

    // Set the vector
    *((uint32_t *)(NVIC_RAM_VECTOR_ADDRESS + (IRQn*4) + (NVIC_USER_IRQ_OFFSET*4))) = vector;
}

uint32_t NVIC_GetVector(IRQn_Type IRQn) {
    uint32_t *vectors = (uint32_t*)NVIC_RAM_VECTOR_ADDRESS;
    // Return the vector
    return vectors[IRQn + 16];
}
#endif //STM32
