#ifndef _FLASH_H_
#define _FLASH_H_
#include "control.h"

typedef struct {
    uint32_t stored_len;
    ADC_Bounds_t adc_bounds;
    ControlParams_t params;
} FlashValues_t;

static const volatile FlashValues_t stored_values __attribute__((section(".config"))) = {
		sizeof(FlashValues_t)
		{

		},
		{

		}
};

FlashValues_t sram_values;


#endif
