#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdint.h>

#define APPS_FAULT_BOUNDS   (1 << 15)
#define APPS_FAULT_DELTA    (1 << 14)
#define APPS_FAULT_PLAUS    (1 << 13)
#define APPS_FAULT_BSE      (1 << 12)

#define APPS_FLAGS_NEG      (1 << 0) /* maybe useful in the future? */

#define APPS_OUT_OF_BOUNDS       6553L
#define APPS_DELTA               6553L
#define APPS_BPS_PLAUS           16384L

#define ROLLING_ADC_FR_POW 4
#define ROLLING_ADC_FRAMES (1 << ROLLING_ADC_FR_POW)
#define ADC_CHANNELS       6
#define ROLLING_ADC_VALS   (ROLLING_ADC_FRAMES * ADC_CHANNELS)

typedef struct {
    uint16_t APPS1;
    uint16_t APPS2;
    uint16_t APPS3;
    uint16_t APPS4;
    uint16_t FBPS;
    uint16_t RBPS;
} ADC_Block_t;

typedef struct {
    uint16_t APPS1_l;
    uint16_t APPS1_h;
    uint16_t APPS2_l;
    uint16_t APPS2_h;
    uint16_t APPS3_l;
    uint16_t APPS3_h;
    uint16_t APPS4_l;
    uint16_t APPS4_h;
    uint16_t BPS_f_min;
    uint16_t BPS_r_min;
} ADC_Bounds_t;

typedef struct {
    uint16_t APPS1_strt;
    uint16_t APPS2_strt;
    uint16_t APPS3_strt;
    uint16_t APPS4_strt;
    int32_t APPS1_mult;
    int32_t APPS2_mult;
    int32_t APPS3_mult;
    int32_t APPS4_mult;
    uint16_t BPS_f_min;
    uint16_t BPS_r_min;
} ADC_Mult_t;

typedef struct {
    uint16_t max_torque;
    uint16_t _reserved;
    uint16_t _reserved2;
    uint16_t brake_threashold;
} ControlParams_t;

typedef struct {
    uint16_t flags;
    uint16_t torque;
} TorqueReq_t;

void ADC_Init();
ADC_Mult_t get_adc_multiplers(ADC_Bounds_t* bounds);

ADC_Block_t condense();
TorqueReq_t calc_torque_request(ADC_Mult_t* bounds, ControlParams_t* params);

#endif
