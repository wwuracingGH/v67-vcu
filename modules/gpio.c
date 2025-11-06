#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "gpio.h"
#include "stm32h5xx.h"

GPIO_TypeDef * buttonPorts[] = _BUTTON_PORTS;
uint32_t buttonPins[] = _BUTTON_PINS;

enum Pin_Mode {
    MODE_INPUT   = 0,
    MODE_OUTPUT  = 1,
    MODE_ALTFUNC = 2,
    MODE_ANALOG  = 3
};

enum PuPd_Mode {
    PUPD_NORMAL  = 0,
    PUPD_PULLUP  = 1,
    PUPD_PULLDN  = 2,
    PUPD_INVALID = 3
};

/* supports up to 8 buttons ig lol */
uint8_t buttonRawInputMask = 0;
uint8_t buttonPressedMask  = 0;
uint8_t buttonReleasedMask = 0;
uint8_t buttonConsumedMask = 0; /* possibly remove? */

#if (INPUT_BUTTON_NUM > 8)
#error INPUT DRIVER ERROR: MAX BUTTONS SUPPORTED IS 8
#endif
uint32_t buttonStartPressedTime[INPUT_BUTTON_NUM];

void GPIO_init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; /* turn on gpio clocks */
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
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE13_Pos)  /* SWD IO        */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE14_Pos)  /* SWD CLK       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODE15_Pos); /* IDK - keep this tho */

    GPIOA->PUPDR  = 0;
    GPIOA->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEED6_Pos); /* Timing pin should be high speed */
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

/* registers all of the events */
void GPIO_Update(uint32_t timestamp) {
    buttonPressedMask = 0;
    buttonReleasedMask = 0;
    for (int i = 0; i < INPUT_BUTTON_NUM; i++) {
        int pressed = (buttonPorts[i]->IDR & 1UL << buttonPins[i]);
        if (!buttonRawInputMask && pressed) {
            buttonPressedMask  |= 1 << i;
            buttonRawInputMask |= 1 << i;
            buttonStartPressedTime[i] = timestamp;
        } else if (buttonRawInputMask && !pressed) {
            buttonRawInputMask &= ~(1 << i);
            buttonConsumedMask &= ~(1 << i);
            buttonReleasedMask |= 1 << i;
        }
    }
}

/* returns if the button was pressed that frame */
int GPIO_buttonPressed(int button_id) {
    return (buttonPressedMask & (1 << button_id)) && !(buttonConsumedMask & (1 << button_id));
}

/* returns if the button was just released that frame */
int GPIO_buttonReleased(int button_id) {
    return (buttonReleasedMask & (1 << button_id)) && !(buttonConsumedMask & (1 << button_id));
}

/* returns the last held time, or -1 if it isn't being held */
int GPIO_buttonHeldTime(int button_id, uint32_t timestamp) {
    if ((buttonConsumedMask | ~buttonRawInputMask) & (1 << button_id)) return -1;
    return buttonStartPressedTime[button_id] - timestamp;
}

/* pretend the button has already been released */
void GPIO_buttonConsume(int button_id) {
    buttonConsumedMask |= 1 << button_id;
}

/* pass in hex color value */
void GPIO_setLED(uint32_t color) {
    uint32_t alpha = (color >> 0) & 0xFF;

    /* generate p15 fixed point colors */
    uint32_t red   = ((color >> 24) & 0xFF) * alpha;
    uint32_t green = ((color >> 16) & 0xFF) * alpha;
    uint32_t blue  = ((color >>  8) & 0xFF) * alpha;

    /* TODO: proper timer implementation */
    GPIOC->ODR &= ~(0b111 << 13);
    GPIOC->ODR |= ((red > 25500) << 13) | ((green > 25500) << 14) | ((blue > 25500) << 15);
}
