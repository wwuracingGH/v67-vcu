//nicole swierstra - vcu code
//as of right now this is just a prototype there's a lot of debugging I'm gonna need to do

#include <stdint.h>
#define STM32F042x6
#include "vendor/CMSIS/Device/ST/STM32F0/Include/stm32f0xx.h"

/**
 * POT_RESOLUTION = 4096 for 5V
 * APPS_MIN_VALUE = 0.5V
 * APPS_MAX_VALUE = 4.5V
 */
#define POT_RESOLUTION (1 << 12)
#define APPS_MIN_VALUE (POT_RESOLUTION / 10)
#define APPS_MAX_VALUE (POT_RESOLUTION - APPS_MIN_VALUE)

#define ADC_TO_DMA

struct {
    volatile uint16_t APPS2;
    volatile uint16_t RBPS;
    volatile uint16_t FBPS;
    volatile uint16_t APPS1;
} ADC_Vars;

void clock_init();
void ADC_DMA_Init();

int apps_fault();
void make_torque_req();

void main(){
    //setup
    clock_init();
    SystemCoreClockUpdate();
    
    ADC_DMA_Init(&ADC_Vars.APPS2, 4); //Bad practice
    while(1) {
        if(apps_fault()) ;
    }
}   

void clock_init() //see clocks.png
{
    //wait one clock cycle before accessing flash memory @48MHZ
    FLASH->ACR |= FLASH_ACR_LATENCY;

    // Enables HSE oscillator
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    //don't technically need either of these, but sets prediv for both (useful when transfering to different stm)
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1 | RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1;

    //Set up PLL
    RCC->CFGR 	|= RCC_CFGR_PLLSRC;
    RCC->CFGR 	|= RCC_CFGR_PLLMUL6;
    RCC->CR     |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Enable PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));
}

void ADC_DMA_Init(uint16_t *dest, uint16_t size){
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    ADC1->CFGR1 &= ~(uint32_t)0b011000; // set 12 bit precision
    
    ADC1->CFGR1 |= ADC_CFGR1_CONT; //analog to digital converter to cont mode
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; //align bits to the right

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN & ADC_CFGR1_DMACFG; //enable dma & make cont

    ADC1->SMPR |= 0b011; //28.5 adc clock cycles

    ADC1->CHSELR |= ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL8; //channels to scan

    GPIOA->MODER |= (3 << GPIO_MODER_MODER1_Pos); // channel 1
    GPIOA->MODER |= (3 << GPIO_MODER_MODER5_Pos); // channel 5
    GPIOA->MODER |= (3 << GPIO_MODER_MODER6_Pos); // channel 6
    GPIOB->MODER |= (3 << GPIO_MODER_MODER0_Pos); // channel 8

    ADC1->CR |= ADC_CR_ADEN; //enable
	
	while(!(ADC1->ISR & 0x1)); //wait for enabled

#ifdef ADC_TO_DMA

    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM; //peripheral to memory
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC; //c
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; //
    
    DMA1_Channel1->CPAR = ADC1->DR; //sets source of dma transfer
    DMA1_Channel1->CMAR = dest; //sets destination of dma transfer
    DMA1_Channel1->CNDTR = size; //sets size of dma transfer
    
    DMA1_Channel1->CCR |= DMA_CCR_EN; //enables dma

    ADC1->ISR = 0; //clear isr register
	ADC1->CR |= ADC_CR_ADSTART; //starts adc
#else

#endif
}

//returns 1 if there's an issue
int apps_fault(){
    return 1;
}