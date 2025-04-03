#include <stdint.h>
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"
#include "llrttsos.h"
_RTOS_IMPLEMENTATION_

void clock_init();

void blinky(){
    GPIOA->ODR ^= (1UL << 5);
}

int main(void){
    clock_init();
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(3UL << 10);
    GPIOA->MODER |= (1UL << 10);

    RTOS_init();

    int default_state = RTOS_addState(0,0);
    RTOS_scheduleTask(default_state, blinky, 500);

    RTOS_switchState(default_state);
    

    SysTick_Config(250000);
    __enable_irq();
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


    RCC->CR |= (3 << RCC_CR_HSIDIV_Pos);

    RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1PEN | (2 << RCC_PLL1CFGR_PLL1M_Pos);
    RCC->PLL1CFGR |= (1 << RCC_PLL1CFGR_PLL1SRC_Pos) | (1 << RCC_PLL1CFGR_PLL1RGE_Pos);

    RCC->PLL1DIVR &= ~RCC_PLL1DIVR_PLL1N_Msk;
    RCC->PLL1DIVR |= (124UL << RCC_PLL1DIVR_PLL1N_Pos);

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

