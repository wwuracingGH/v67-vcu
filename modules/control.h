/*
 * Nicole Swierstra
 * V67 Control Algorithms
 * 
 * This file contains definitions and algorithms for calculating driver input and
 * turning it into control
 *
 * ============= ADC Handling ===============
 * ADCs are set up to do continuous conversion with the DMA in the init function
 * 
 * =========== APPS Multipliers =============
 * A multiplication vector is gotten for remapping the apps using fixed point 
 * multiplication, and then stored somewhere else
 * 
 * =========== Torque Request ===============
 * Calculated with fixed point multiplication for maximum speed, <3us. also checks
 * for apps sensor faults
 * 
 * ======= Vehicle Dynamics Modeling ======== 
 * Coming soon, maybe
 */

#ifndef _MODULES_CONTROL_H_
#define _MODULES_CONTROL_H_

#include <stdint.h>

#define APPS_FAULT_MASK     0xFF00
#define APPS_FAULT_BOUNDS   (1 << 15)
#define APPS_FAULT_DELTA    (1 << 14)
#define APPS_FAULT_PLAUS    (1 << 13)
#define APPS_FAULT_BSE      (1 << 12)
#define APPS_FAULT_DISAG    (1 << 11)

#define APPS_FLAGS_NEG      (1 << 0) /* maybe useful in the future? */

#define APPS_OUT_OF_BOUNDS       6553L
#define APPS_MAX_DELTA           6553L
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
    uint16_t BPS_s_min;
    uint16_t BPS_s_max;
    uint16_t BPS_f_bias;  /* p16 proportion - f_val / (f_val + r_val) - calculated by vcu reprogrammer */
} ADC_Bounds_t;

typedef struct {
    uint16_t APPS1_strt;
    uint16_t APPS2_strt;
    uint16_t APPS3_strt;
    uint16_t APPS4_strt;
    int32_t  APPS1_mult;
    int32_t  APPS2_mult;
    int32_t  APPS3_mult;
    int32_t  APPS4_mult;
    uint16_t BPS_b_mult; /* proportion of 300 bar */
    uint16_t BPS_f_mult; /* mult by 1 / fbias */
    uint16_t BPS_r_mult; /* rear brake / total braking pressure in p15 format */
    uint16_t BPS_s_min;  /* sensor minimum */
} ADC_Mult_t;

typedef struct {
    uint16_t max_torque;        /* maximum torque request   - 10x Nm*/
    uint16_t hard_braking;      /* total system pressure for hard braking */
    uint16_t max_braking_pres;  /* maximum braking pressure - 10x bar */
    uint16_t _reserved2;
} ControlParams_t;

typedef struct {
    uint16_t flags;
    uint16_t torque;            /* decimal torque   - 10x Nm */
    uint16_t brake_pressure;    /* decimal pressure - 10x bar */
    uint16_t steer_angle;       /* possibly unused  - 10x deg */
} ControlReq_t;

/* TODO */
typedef struct {
    uint32_t dynamicState;
} VehicleDynamicState_t;

/* TODO */
typedef struct {
    uint32_t dynamicParams;
} VehicleDynamicParams_t;

void CTRL_ADCinit();

/*
 * Returns a multiplier object that can be stored and used for apps calculation
 * 
 * Takes in the bounds to get the multipliers from 
 */
ADC_Mult_t CTRL_getADCMultiplers(volatile ADC_Bounds_t* bounds);

/*
 * Only exposed for debugging, can be hidden if raw apps values aren't necessary
 */
ADC_Block_t CTRL_condense();

/* 
 * Returns the control vector for the car, including:
 * fault flags
 * torque in NM
 * total brake pressure in bar
 * steering angle (maybe)
 */
ControlReq_t CTRL_getCommand(ADC_Mult_t* mult, ControlParams_t* params, uint16_t bound);

#endif
