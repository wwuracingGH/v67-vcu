//nicole swierstra - vcu code
//as of right now this is just a prototype there's a lot of debugging I'm gonna need to do
//and a lot of comments i'm going to need to remove when I have the documentation working lol

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

//will disable this if there's ever a problem with the dma it hasn't earned my trust
#define ADC_TO_DMA

struct {
    volatile uint32_t APPS2;
    volatile uint32_t RBPS;
    volatile uint32_t FBPS;
    volatile uint32_t APPS1;
} ADC_Vars;

struct {
    volatile int ready_to_drive;
} car_state;

enum Pin_Mode {
    MODE_INPUT   = 0b00,
    MODE_OUTPUT  = 0b01,
    MODE_ALTFUNC = 0b10,
    MODE_ANALOG  = 0b11
};

void clock_init();
void ADC_DMA_Init();
void GPIO_Init();

int apps_fault();
void make_torque_req();
void poll_ADC();
void default_handler();
void kill_car();

void main(){
    //setup
    clock_init();
    SystemCoreClockUpdate();
    
    GPIO_Init(); //must be called first
    ADC_DMA_Init(&ADC_Vars.APPS2, 4); //Bad practice 🤷‍♀️
    for(;;) {
        if(apps_fault()) default_handler();

        if(car_state.ready_to_drive){
            request_torque();
        }
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

void ADC_DMA_Init(uint32_t *dest, uint32_t size){
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 &= ~(uint32_t)0b011000; // set 12 bit precision
    
    ADC1->CFGR1 |= ADC_CFGR1_CONT; //analog to digital converter to cont mode
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; //align bits to the right

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN & ADC_CFGR1_DMACFG; //enable dma & make cont

    ADC1->SMPR |= 0b011; //28.5 adc clock cycles

    ADC1->CHSELR |= ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL8; //channels to scan

    ADC1->CR |= ADC_CR_ADEN; //enable
	
    ADC1->ISR = 0; //clear isr register
	while(!(ADC1->ISR & 0x1)); //wait for enabled

#ifdef ADC_TO_DMA

    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM; //peripheral to memory
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC; //c
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; //set size of data to transfer
    
    DMA1_Channel1->CPAR = ADC1->DR; //sets source of dma transfer
    DMA1_Channel1->CMAR = dest; //sets destination of dma transfer
    DMA1_Channel1->CNDTR = size; //sets size of dma transfer
    
    DMA1_Channel1->CCR |= DMA_CCR_EN; //enables dma

	ADC1->CR |= ADC_CR_ADSTART; //starts adc
#else

#endif
}

void GPIO_Init(){
    //turn on gpio clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

    //todo: figure out how to put this into a bitfield struct so it's not at the bottom of the code and sets the register once
    GPIOA->MODER |= (MODE_INPUT     << GPIO_MODER_MODER0_Pos); // set porta 0 as digital input
    GPIOA->MODER |= (MODE_ANALOG    << GPIO_MODER_MODER1_Pos); // set porta 1 as analog input
    GPIOA->MODER |= (MODE_ANALOG    << GPIO_MODER_MODER5_Pos); // set porta 5 as analog input
    GPIOA->MODER |= (MODE_ANALOG    << GPIO_MODER_MODER6_Pos); // set porta 6 as analog input
    GPIOA->MODER |= (MODE_OUTPUT    << GPIO_MODER_MODER9_Pos); // set porta 9 as digital output
    GPIOB->MODER |= (MODE_ANALOG    << GPIO_MODER_MODER0_Pos); // set portb 0 as analog input
    GPIOB->MODER |= (MODE_INPUT     << GPIO_MODER_MODER1_Pos); // set portb 1 as digital input
    GPIOB->MODER |= (MODE_OUTPUT    << GPIO_MODER_MODER3_Pos); // TODO: im sorry what exactly is the bspd light
    GPIOB->MODER |= (MODE_OUTPUT    << GPIO_MODER_MODER5_Pos); // set portb 5 as digital output
    GPIOB->MODER |= (MODE_OUTPUT    << GPIO_MODER_MODER6_Pos); // set portb 6 as digital output
    GPIOB->MODER |= (MODE_OUTPUT    << GPIO_MODER_MODER7_Pos); // set portb 7 as digital output
    //the alternate, evil version:
    //GPIOA->MODER = 0b00000000000001000011110000001100;
    //GPIOB->MODER = 0b00000000000000000101010001000011;
}

//returns 1 if there's an issue
int apps_fault(){

    return 1;
}

//call if error
void kill_car() {
    if (car_state.ready_to_drive){
        car_state.ready_to_drive = 0;
        //TODO: violently rip motor out of ready to drive with a can message
    }

    //turn on led0
    GPIOB->ODR |= GPIO_ODR_6;
}

void request_torque(){

}