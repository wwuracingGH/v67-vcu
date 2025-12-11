#include "flash.h"
#include "stm32h533xx.h"
#include <stdint.h>

const volatile CarParameters_t stored_values __attribute__((section(".config"))) = {
        sizeof(CarParameters_t),
        { 1723, 681, 2322, 3361, 2031, 1023, 2055, 3056, 409, 3686, 35000 },
		{ 1000, 100, 3000, 0},
        {
            10000000
        }
};

volatile CarParameters_t ram_values = { 0 };

int ram_initialized = 0;

/* 
 * Unlocks the flash to be able to write to it 
 */
void FLASH_Init() {
  	FLASH->NSKEYR = 0x45670123;
	FLASH->NSKEYR = 0xCDEF89AB;
};

/*
 * Takes in a location in device sram, a page # in high cycle flash, and writes one to the other
 * First clearing the memory with page erase, and then writing the new data from src.
 */
void FLASH_WriteSector(void* src, uint32_t page_start, uint32_t len) {
	while((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)){}; /* Wait for flash to be avaliable */

	if (FLASH->NSCR & FLASH_CR_LOCK) { /* Unlock the flash if it's locked */
		FLASH_Init();
	}

	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

	FLASH->NSCR |= FLASH_CR_BKSEL; /* Select bank 2 */

	FLASH->NSCR |= FLASH_CR_SER; /* Tell flash we're doing a sector erase */

	FLASH->NSCR &= ~FLASH_CR_SNB_Msk; /* Clear sector selection bits */

	uint32_t sector = 31 - page_start;

	FLASH->NSCR |= ((sector << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk); /* tell flash which sector to erase */

	FLASH->NSCR |= FLASH_CR_START; /* Start the erase */

	while(FLASH->NSSR & FLASH_SR_BSY){}; /* wait for the erase to finish */
 
	FLASH->NSCR &= ~FLASH_CR_SER_Msk; /* Tell the flash we're no longer doing a sector erase */

	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

	if (!(FLASH->NSCR & FLASH_CR_PG)) { /* Tell the flash we're gonna program if we haven't yet */
		FLASH->NSCR |= FLASH_CR_PG;
	}

	uint32_t* page_mem_addr = ((uint32_t*)0x0900C000) + (page_start * 0x1800U); /*Bank 2 base + page_num * Page size */

	for (uint32_t i = 0; i < len / 4; i++) {
		page_mem_addr[i] = ((uint32_t*)src)[i];
	}

	while(FLASH->NSSR & FLASH_SR_BSY){}; /* wait for the write to finish */

	FLASH->NSCR &= ~FLASH_CR_PG_Msk; /* clear programming bit */

	FLASH->NSCR |= FLASH_CR_LOCK; /* Lock the flash while we're not using it */
};

void FLASH_EraseHighCycle() {
    while((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)){}; /* Wait for flash to be avaliable */

    if (FLASH->NSCR & FLASH_CR_LOCK) { /* Unlock the flash if it's locked */
	    FLASH_Init();
    }

    FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */
    
    for (int i = 24; i < 32; i++) {
    	FLASH->NSCR |= FLASH_CR_BKSEL; /* Select bank 2 */

    	FLASH->NSCR |= FLASH_CR_SER; /* Tell flash we're doing a sector erase */

    	FLASH->NSCR &= ~FLASH_CR_SNB_Msk; /* Clear sector selection bits */

    	FLASH->NSCR |= ((i << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk); /* tell flash which sector to erase */

    	FLASH->NSCR |= FLASH_CR_START; /* Start the erase */

    	while(FLASH->NSSR & FLASH_SR_BSY); /* wait for the erase to finish */

    	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */
    }
	
    FLASH->NSCR &= ~FLASH_CR_SER; /* Clear sector erase bit. */

    while((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)); /* Wait for flash to be avaliable */

    FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

    if (FLASH->NSCR & FLASH_CR_LOCK) { /* Unlock the flash if it's locked */
    	FLASH_Init();
    }
    
    if (!(FLASH->NSCR & FLASH_CR_PG)) { /* Tell the flash we're gonna program if we haven't yet */
	    FLASH->NSCR |= FLASH_CR_PG;
    }

    uint32_t* flash_start = (uint32_t*)0x0900C000;
    volatile uint32_t total_words = 0xC000 / 4; /* 0x3000 */
    for(uint32_t i = 0; i < total_words; i++) {
    	flash_start[i] = 0xFFFFFFFF;
    	while((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)){}; /* Wait for flash to be avaliable */
    }	

    FLASH->NSCR &= ~FLASH_CR_PG_Msk; /* clear programming bit */

    FLASH->NSCR |= FLASH_CR_LOCK; /* Lock the flash while we're not using it */
};

volatile CarParameters_t* FLASH_getVals(){
	uint16_t* sv_ptr = (uint16_t*)&stored_values;
	uint16_t* rm_ptr = (uint16_t*)&ram_values;

	const int writes = sizeof(CarParameters_t) / 2;
	if (!ram_initialized) {
		for(int i = 0; i < writes; i++){
			rm_ptr[i] = sv_ptr[i];
		}
	}

	return &ram_values;
}

int FLASH_getVal(int id){
	switch(id) {
		case FLASH_PARAMID_APPS1_L: 
			return (int)ram_values.adc_bounds.APPS1_l;
			break;
		case FLASH_PARAMID_APPS1_H: 
			return (int)ram_values.adc_bounds.APPS1_h;
			break;
		case FLASH_PARAMID_APPS2_L: 
			return (int)ram_values.adc_bounds.APPS2_l;
			break;
		case FLASH_PARAMID_APPS2_H: 
			return (int)ram_values.adc_bounds.APPS2_h;
			break;
		case FLASH_PARAMID_APPS3_L: 
			return (int)ram_values.adc_bounds.APPS3_l;
			break;
		case FLASH_PARAMID_APPS3_H: 
			return (int)ram_values.adc_bounds.APPS3_l;
			break;
		case FLASH_PARAMID_APPS4_L: 
			return (int)ram_values.adc_bounds.APPS4_l;
			break;
		case FLASH_PARAMID_APPS4_H: 
			return (int)ram_values.adc_bounds.APPS4_h;
			break;
		case FLASH_PARAMID_BPS_S_MIN: 
			return (int)ram_values.adc_bounds.BPS_s_min;
			break;
		case FLASH_PARAMID_BPS_S_MAX: 
			return (int)ram_values.adc_bounds.BPS_s_max;
			break;
		case FLASH_PARAMID_BPS_F_BIAS:
			return (int)ram_values.adc_bounds.BPS_f_bias; 
			break;
		case FLASH_PARAMID_MAX_TORQUE: 
			return (int)ram_values.params.max_torque;
			break;
		case FLASH_PARAMID_HARD_BRAKING: 
			return (int)ram_values.params.hard_braking;
			break;
		case FLASH_PARAMID_MAX_BRAKE_P: 
			return (int)ram_values.params.max_braking_pres;
			break;
	}
	return -1;
}

void FLASH_storeVal(int id, int newVal, int write){
	switch(id) {
		case FLASH_PARAMID_APPS1_L: 
			ram_values.adc_bounds.APPS1_l = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS1_H: 
			ram_values.adc_bounds.APPS1_h = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS2_L: 
			ram_values.adc_bounds.APPS2_l = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS2_H: 
			ram_values.adc_bounds.APPS2_h = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS3_L: 
			ram_values.adc_bounds.APPS3_l = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS3_H: 
			ram_values.adc_bounds.APPS3_l = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS4_L: 
			ram_values.adc_bounds.APPS4_l = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_APPS4_H: 
			ram_values.adc_bounds.APPS4_h = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_BPS_S_MIN: 
			ram_values.adc_bounds.BPS_s_min = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_BPS_S_MAX: 
			ram_values.adc_bounds.BPS_s_max = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_BPS_F_BIAS:
			ram_values.adc_bounds.BPS_f_bias = (uint16_t)newVal; 
			break;
		case FLASH_PARAMID_MAX_TORQUE: 
			ram_values.params.max_torque = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_HARD_BRAKING: 
			ram_values.params.hard_braking = (uint16_t)newVal;
			break;
		case FLASH_PARAMID_MAX_BRAKE_P: 
			ram_values.params.max_braking_pres = (uint16_t)newVal;
			break;
		default: break;
	}

	if (write) {
		FLASH_WriteSector(&ram_values, 0, sizeof(ram_values));
	}
}
