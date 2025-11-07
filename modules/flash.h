#ifndef _MODULES_FLASH_H_
#define _MODULES_FLASH_H_
#include "control.h"

typedef struct {
    uint32_t stored_len;
    ADC_Bounds_t adc_bounds;
    ControlParams_t params;
} CarParameters_t;

static const volatile CarParameters_t stored_values __attribute__((section(".config"))) = {
        sizeof(CarParameters_t),
        {
            0
        },
        {
            0
        }
};

/* 
 * Unlocks the flash to be able to write to it 
 */
void FLASH_Init();

/*
 * Takes in a location in device sram, a page # in high cycle flash, and writes one to the other
 * First clearing the memory with page erase, and then writing the new data from src.
 */
void FLASH_WriteSector(void* src, uint32_t page_start, uint32_t len);

/*
 * Clears all high cycle data.
 */
void FLASH_EraseHighCycle();

#endif
