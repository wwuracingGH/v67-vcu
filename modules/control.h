#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdint.h>

#define APPS_FAULT_BOUNDS   (1 << 15)
#define APPS_FAULT_DELTA    (1 << 14)
#define APPS_FAULT_PLAUS    (1 << 13)
#define APPS_FAULT_BSE      (1 << 12)

#define APPS_FLAGS_NEG      (1 << 0) /* maybe useful in the future? */

#define APPS_OUT_OF_BOUNDS       0.1f

#define ROLLING_ADC_FR_POW 2
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
    uint16_t BPS_min;
    uint16_t BPS_max;
} ADC_Bounds_t;

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

ADC_Block_t condense();
void ADC_Init();
TorqueReq_t calc_torque_request(ADC_Bounds_t bounds, ControlParams_t params);

#endif
