#include "gpio.h"
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "stm32h5xx.h"

enum Pin_Mode {
    MODE_INPUT   = 0b00,
    MODE_OUTPUT  = 0b01,
    MODE_ALTFUNC = 0b10,
    MODE_ANALOG  = 0b11
};

enum PuPd_Mode {
    PUPD_NORMAL  = 0b00,
    PUPD_PULLUP  = 0b01,
    PUPD_PULLDN  = 0b10,
    PUPD_INVALID = 0b11
};

void GPIO_Init(){
    /* turn on gpio clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; 
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; 
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; 
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;

    GPIOA->MODER  = (MODE_ANALOG    << GPIO_MODER_MODE0_Pos)   /* APPS0         */
                 |  (MODE_ANALOG    << GPIO_MODER_MODE1_Pos)   /* APPS1         */
                 |  (MODE_ANALOG    << GPIO_MODER_MODE2_Pos)   /* APPS2         */
                 |  (MODE_ANALOG    << GPIO_MODER_MODE3_Pos)   /* APPS3         */
                 |  (MODE_ANALOG    << GPIO_MODER_MODE4_Pos)   /* RBPS          */
                 |  (MODE_ANALOG    << GPIO_MODER_MODE5_Pos)   /* FBPS          */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODE6_Pos)   /* TIMING PIN    */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE9_Pos)   /* UART TX       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE10_Pos)  /* UART RX       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE11_Pos)  /* CAN1 TX       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE12_Pos)  /* CAN1 RX       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE13_Pos)  /* SWD IO 		*/
				 |  (MODE_ALTFUNC   << GPIO_MODER_MODE14_Pos)  /* SWD CLK 		*/
				 |  (MODE_ALTFUNC   << GPIO_MODER_MODE15_Pos); /* IDK - keep this tho */


    GPIOA->PUPDR  = 0;
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED6_Pos); /* Timing pin should be high speed */
    GPIOA->AFR[1] |= (7UL << GPIO_AFRH_AFSEL9_Pos) | /* USART TX */
                    (7UL << GPIO_AFRH_AFSEL10_Pos) | /* USART RX */
                    (9UL << GPIO_AFRH_AFSEL11_Pos) | /* CAN1 TX  */
                    (9UL << GPIO_AFRH_AFSEL12_Pos);  /* CAN1 RX  */

    GPIOB->MODER &= (0xFFFFUL << 0x10UL);
    GPIOB->MODER |= (MODE_ALTFUNC   << GPIO_MODER_MODE5_Pos)   /* CAN2 TX       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE6_Pos)   /* CAN2 RX       */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODE10_Pos); /* BUZZER        */
    
    GPIOB->AFR[0] |= (9 << GPIO_AFRL_AFSEL5_Pos) | /* CAN2 TX */
                     (9 << GPIO_AFRL_AFSEL6_Pos);  /* CAN2 RX */

    GPIOB->PUPDR  = 0;
    
    GPIOC->MODER = ~((MODE_ALTFUNC    << GPIO_MODER_MODE13_Pos) | /* STATUS LED R */
                      (MODE_ALTFUNC    << GPIO_MODER_MODE14_Pos) | /* STATUS LED G */
                      (MODE_ALTFUNC    << GPIO_MODER_MODE15_Pos)); /* STATUS LED B */
    
    GPIOH->MODER &= (MODE_INPUT     << GPIO_MODER_MODE0_Pos);  /* RTD BUTTON    */
    GPIOH->PUPDR |= (PUPD_PULLUP    << GPIO_PUPDR_PUPD0_Pos);  /* Button is pulled up */
}
