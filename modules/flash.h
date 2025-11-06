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



#endif
