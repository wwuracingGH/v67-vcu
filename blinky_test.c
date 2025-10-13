#include <stdint.h>
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"
#include "llrttsos.h"
#include "modules/gpio.h"
#include "modules/logging.h"
#include "vendor/printf/printf.h"
_RTOS_IMPLEMENTATION_

#define SYS_CLOCK 250000000UL

void clock_init();

void blinky(){
	static int col = 0;
    GPIOC->ODR &= ~(0b111 << 13);
    switch(col){
    case 0: GPIOC->ODR |= 0b100 << 13; break;
    case 1: GPIOC->ODR |= 0b110 << 13; break;
    case 2: GPIOC->ODR |= 0b010 << 13; break;
    case 3: GPIOC->ODR |= 0b011 << 13; break;
    case 4: GPIOC->ODR |= 0b001 << 13; break;
    default: GPIOC->ODR |= 0b101 << 13; break;
    }

    col++;
    col %= 6;
}

void say_hello(){
    LOG("Hello World!\n");
}

int main(void) {
    clock_init();
    GPIO_Init();
    RTOS_init();
    logging_init();
    _enable_logging_all();

    LOGLN("STARTING STM");

    int default_state = RTOS_addState(0,0);
    RTOS_scheduleTask(default_state, blinky, 100);
    RTOS_scheduleTask(default_state, say_hello, 500);

    RTOS_switchState(default_state);

    RTOS_start_armeabi(SYS_CLOCK);

    LOGLN("CLOCK CHECK: %d", SysTick->LOAD + 1);
    while(1){
        RTOS_ExecuteTasks();
    }
}

void clock_init() /* turns up the speed to 250mhz */
{
    /* use voltage scaling profile 0 */
    PWR->VOSCR |= (0b11 << PWR_VOSCR_VOS_Pos); 
    /* change the flash access latency */
    FLASH->ACR |= 5 << FLASH_ACR_LATENCY_Pos;
    /* change the flash delay */
    FLASH->ACR |= 2 << FLASH_ACR_WRHIGHFREQ_Pos;
    
    /* prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTEN;

    RCC->CR |= (3 << RCC_CR_HSIDIV_Pos);

    RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1PEN | (2 << RCC_PLL1CFGR_PLL1M_Pos);
    RCC->PLL1CFGR |= (1 << RCC_PLL1CFGR_PLL1SRC_Pos) | (1 << RCC_PLL1CFGR_PLL1RGE_Pos);

    RCC->PLL1DIVR &= ~RCC_PLL1DIVR_PLL1N_Msk;
    RCC->PLL1DIVR |= ((((SYS_CLOCK/1000000UL) >> 1) - 1) << RCC_PLL1DIVR_PLL1N_Pos);

    /* Enables PLL1 and locks values */
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY));

    /* sets system clock source as PLL1 */
    RCC->CFGR1 |= 0b11 << RCC_CFGR1_SW_Pos;
    while (!(RCC->CFGR1 & (0b11 << RCC_CFGR1_SWS_Pos)));
}

void systick_handler(){ 
    RTOS_Update();
}
