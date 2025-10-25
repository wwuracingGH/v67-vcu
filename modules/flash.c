#include "flash.h"
#include "stm32h533xx.h"
#include <stdint.h>
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

	FLASH->NSCR |= ((page_start << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk); /* tell flash which sector to erase */

	FLASH->NSCR |= FLASH_CR_START; /* Start the erase */

	while(FLASH->NSSR & FLASH_SR_BSY){}; /* wait for the erase to finish */

	FLASH->NSCR &= ~FLASH_CR_SER_Msk; /* Tell the flash we're no longer doing a sector erase */

	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

	if (!(FLASH->NSCR & FLASH_CR_PG)) { /* Tell the flash we're gonna program if we haven't yet */
		FLASH->NSCR |= FLASH_CR_PG;
	}

	uint32_t* page_mem_addr = (uint32_t*) 0x09017FFF + (page_start * 0x60000U); /*Bank 2 base + page_num * Page size */

	for(uint32_t i = 0; i < len/4; i++) {
		page_mem_addr[i] = ((uint32_t*)src)[i];
	}

	while(FLASH->NSSR & FLASH_SR_BSY){}; /* wait for the write to finish */

	FLASH->NSCR &= ~FLASH_CR_PG_Msk; /* clear programming bit */

	FLASH->NSCR |= FLASH_CR_LOCK; /* Lock the flash while we're not using it */
};
