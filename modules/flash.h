#ifndef _FLASH_H_
#define _FLASH_H_
#include <stdint.h>

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
