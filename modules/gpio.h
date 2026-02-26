/*
 * Nicole Swierstra
 * V67 GPIO/Input Logic
 * 
 * The purpose of this module is to initialize and take data directly from pins.
 * 
 * This includes:
 * 
 * ========== Extensible input system ==========
 * I wanted a way of turning off the car that felt natural -> 
 * holding the button down for 1 s turns off the car reguardless of if you release it afterwards or not
 * 
 * I wanted a way to consume that input so it wouldn't then put the car back into ready to drive as soon as it switched states
 * 
 * I also wanted a short bump of the rtd button to be registered as "different" than the long press to avoid premature triggering
 * 
 * In the future, supports for knobs and dials might be added
 * 
 * ========== RTD light control ==========
 * The light is on timer pins to allow it to be set to any of 4.3 billion colors (theoretically).
 * 
 * The idea is to define a hex color for every state and allow passing that in as a single parameter
 *
 * ========== GPIO init ============
 * 
 * Tells the pins what to do 
 */

#ifndef _MODULES_GPIO_H_
#define _MODULES_GPIO_H_

#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "stm32h5xx.h"

extern GPIO_TypeDef *   buttonPorts[];
extern uint8_t          buttonPins[];
extern uint8_t          buttonAcLow[];
extern const uint8_t    buttonNum;

/* init gpio ports and rtd LED */
void GPIO_init();

/* registers all of the events */
void GPIO_Update(uint32_t curr_timestamp);

/* returns if the button was pressed that frame */
int GPIO_buttonPressed(int button_id);

/* returns if the button was just released that frame */
int GPIO_buttonReleased(int button_id);

/* returns the last held time, or -1 if it hasn't been held */
int GPIO_buttonHeldTime(int button_id, uint32_t curr_timestamp);

/* pretend the button has already been released */
void GPIO_buttonConsume(int button_id);

/* pass in hex color value RGBL where L is brightness */
void GPIO_setLED(uint32_t color);

#endif
