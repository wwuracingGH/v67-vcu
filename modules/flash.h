#ifndef _MODULES_FLASH_H_
#define _MODULES_FLASH_H_
#include "control.h"

typedef struct {
    uint32_t stored_len;
    ADC_Bounds_t adc_bounds;
    ControlParams_t params;
    VehicleDynamicParams_t vdp;
} CarParameters_t;

#define FLASH_PARAMID_APPS1_L 		0x0
#define FLASH_PARAMID_APPS1_H 		0x1
#define FLASH_PARAMID_APPS2_L 		0x2
#define FLASH_PARAMID_APPS2_H 		0x3
#define FLASH_PARAMID_APPS3_L 		0x4
#define FLASH_PARAMID_APPS3_H 		0x5
#define FLASH_PARAMID_APPS4_L 		0x6
#define FLASH_PARAMID_APPS4_H 		0x7
#define FLASH_PARAMID_BPS_S_MIN		0x8
#define FLASH_PARAMID_BPS_S_MAX     0x9
#define FLASH_PARAMID_BPS_F_BIAS    0xA
#define FLASH_PARAMID_MAX_TORQUE    0xC
#define FLASH_PARAMID_HARD_BRAKING  0xD
#define FLASH_PARAMID_MAX_BRAKE_P   0xE

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

/*
 * Returns a reference to the values stored in SRAM
 */
volatile CarParameters_t* FLASH_getVals();
int FLASH_getVal();

/*
 * stores the value newVal in the value with code id in ram
 */
void FLASH_storeVal(int id, int newVal, int write);

#endif
