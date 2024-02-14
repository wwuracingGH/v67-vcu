//nicole swierstra - vcu code
//as of right now this is just a prototype there's a lot of debugging I'm gonna need to do
//and a lot of comments i'm going to need to remove when I have the documentation working lol

#include <stdint.h>
#ifndef STM32F042x6
#define STM32F042x6
#endif
#include "vendor/CMSIS/Device/ST/STM32F0/Include/stm32f0xx.h"
#include "canDefinitions.h"
#include "vendor/qfplib/qfplib.h"

/**
 * POT_RESOLUTION = 4096 for 5V
 * APPS_MIN_VALUE = 0.5V
 * APPS_MAX_VALUE = 4.5V
 */
#define POT_RESOLUTION      4096
#define APPS1_MIN_FRAC      0.6
#define APPS1_MAX_FRAC      0.8
#define APPS2_MIN_FRAC      0.4
#define APPS2_MAX_FRAC      0.6

#define MIN_TORQUE_REQ      0 //do not change this. car not legally allowed to go backwards.
#define MAX_TORQUE_REQ      5

#define BRAKES_THREASHOLD   500 //change this in the future

#define CONSTINV(n)             (1.0f / (float)(n)) //hopefully is forced to compile to a constant float with const variables
#define REMAP0_1(n, min, max)   ((float)(n - min) / CONSTINV(max - min))
#define REMAPm_M(n, min, max)   ((n) * (max - min) + (min))
#define FABS(x)                 ((x) > 0.0f ? -(x) : (x))

const uint32_t APPS1_MIN    = 5;
const uint32_t APPS1_MAX    = 0;
const uint32_t APPS2_MIN    = 0;
const uint32_t APPS2_MAX    = 0;
const uint32_t FBPS_MIN     = 0;
const uint32_t FBPS_MAX     = 0;
const uint32_t RBPS_MIN     = 0;
const uint32_t RBPS_MAX     = 0;

struct {
    volatile uint16_t APPS2;
    volatile uint16_t RBPS;
    volatile uint16_t FBPS;
    volatile uint16_t APPS1;
} ADC_Vars;

struct {
    volatile uint16_t ready_to_drive;
    volatile uint16_t torque_req;
    union {
        struct MC_HighSpeed hs;
        uint64_t bits;
    } hsmessage;
} car_state;

typedef struct _canmsg{
    volatile uint32_t id;
    volatile uint32_t len;
    volatile uint64_t data;
} CAN_msg;

enum Pin_Mode {
    MODE_INPUT   = 0b00,
    MODE_OUTPUT  = 0b01,
    MODE_ALTFUNC = 0b10,
    MODE_ANALOG  = 0b11
};



void clock_init();
void ADC_DMA_Init();
void GPIO_Init();
void CAN_Init();

void default_handler();

int APPS_calc();
void send_CAN(uint16_t, uint8_t, uint8_t*);
void process_can(CAN_msg);

/* 

TODO: what is thumb 2 and why does it hate me :/
uint32_t __aeabi_uidiv(uint32_t u, uint32_t v) {
    uint32_t q = 0, k = 0;
    asm(
        "clz r4, %[n]                   \n\t"
        "clz r5, %[d]                   \n\t"
        "sub %[cnt], r4, r5             \n\t"
        "lsl %[d], %[d], %[cnt]         \n\t"
        "mov %[ret], #0                 \n\t"
        "lsl %[cnt], #1, %[cnt]         \n"
    ".loop                              \n\t"
        "cmp %[n], %[d]                 \n\t"
        "subls %[n], %[n], %[d]         \n\t"
        "addls %[ret], %[cnt], %[ret]   \n\t"
        "lsr %[d], %[d], #1             \n\t"
        "lsrs %[cnt], %[cnt], #1        \n\t"
        "beq loop"
    :   [ret]"=r" (q)
    :   [cnt]"+r" (k), [n]"+r" (u), [d]"+r" (v)
    :   "cc"
    );

    return q;
}
*/

uint32_t clz(uint32_t i){
    uint32_t j = 0, n = i;
    while((n = n >> 1)) j++;
    return j;
}

uint32_t __aeabi_uidiv(uint32_t u, uint32_t v) {
    uint32_t q = 0, k = clz(u) - clz(v);
    v << k;
    k = 1 << k;
    do {
        if(v >= u) continue;
        u -= v;
        q += k;
    }
    while(v = v >> 1, (k = k >> 1));
    return q;
}

int main(){
    //setup
    clock_init();
    SystemCoreClockUpdate();
    
    GPIO_Init(); //must be called first

    GPIOB->ODR |= 1 << 7;
    ADC_DMA_Init(&ADC_Vars.APPS2, 4); //Bad practice 🤷‍♀️
    CAN_Init();

    __enable_irq(); //enable interrupts

    uint32_t canTimer = 500000;

    for(;;) {
        APPS_calc(&car_state.torque_req);


        if(!(canTimer--)){
            canTimer = 500000;
            send_CAN(0b0001, 8, (uint8_t*)&ADC_Vars.APPS2);
        }
    }
}   

void clock_init() //see clocks.png
{
    //wait one clock cycle before accessing flash memory @48MHZ
    FLASH->ACR |= FLASH_ACR_LATENCY;

    // Enables HSE oscillator
    RCC->CR  |= RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->CIR |= RCC_CIR_HSERDYC;
    RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_0);

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

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG; //enable dma & make cont

    ADC1->SMPR |= 0b011; //28.5 adc clock cycles

    ADC1->CHSELR |= ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL8; //channels to scan

    RCC->CR2 |= RCC_CR2_HSI14ON;
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
    ADC1->CFGR2 &= (~ADC_CFGR2_CKMODE);

    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
        ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);

    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM; //peripheral to memory
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_TEIE; //c
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; //set size of data to transfer
    
    DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); //sets source of dma transfer
    DMA1_Channel1->CMAR = (uint32_t)dest; //sets destination of dma transfer
    DMA1_Channel1->CNDTR = size; //sets size of dma transfer
    
    DMA1_Channel1->CCR |= DMA_CCR_EN; //enables dma

	ADC1->CR |= ADC_CR_ADSTART; //starts adc

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* (1) */
    NVIC_SetPriority(DMA1_Channel1_IRQn,0); /* (2) */
}

void CAN_Init (){
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;
    CAN->MCR |= CAN_MCR_INRQ;

    while (!(CAN->MSR & CAN_MSR_INAK));
    
    CAN->MCR &= ~CAN_MCR_SLEEP;
    while (CAN->MSR & CAN_MSR_SLAK);

    CAN->BTR |= 3 << CAN_BTR_BRP_Pos | 1 << CAN_BTR_TS1_Pos | 0 << CAN_BTR_TS2_Pos;
    CAN->MCR &= ~CAN_MCR_INRQ;
    
    while (CAN->MSR & CAN_MSR_INAK);

    CAN->FMR |= CAN_FMR_FINIT;
    CAN->FA1R |= CAN_FA1R_FACT0;
    CAN->sFilterRegister[0].FR1 = 0; // Its like a filter, but doesn't filter anything!
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->FMR &=~ CAN_FMR_FINIT;
    CAN->IER |= CAN_IER_FMPIE0;
}

void GPIO_Init(){
    //turn on gpio clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

    GPIOA->MODER |= (MODE_INPUT     << GPIO_MODER_MODER0_Pos)   // set porta 0 as digital input
                 |  (MODE_ANALOG    << GPIO_MODER_MODER1_Pos)   // set porta 1 as analog input
                 |  (MODE_ANALOG    << GPIO_MODER_MODER5_Pos)   // set porta 5 as analog input
                 |  (MODE_ANALOG    << GPIO_MODER_MODER6_Pos)   // set porta 6 as analog input
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER9_Pos)   // set porta 9 as digital output
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODER11_Pos)  //can TX
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODER12_Pos); //can RX

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12); // no pull up - pull down on can
    GPIOA->AFR[1] = (4 << GPIO_AFRH_AFSEL11_Pos) | (4 << GPIO_AFRH_AFSEL12_Pos); //set up can

    GPIOB->MODER |= (MODE_ANALOG    << GPIO_MODER_MODER0_Pos)   // set portb 0 as analog input
                 |  (MODE_INPUT     << GPIO_MODER_MODER1_Pos)   // set portb 1 as digital input
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER5_Pos)   // set portb 5 as digital output
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER6_Pos)   // set portb 6 as digital output
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER7_Pos);  // set portb 7 as digital output
}

//returns 1 if there's an issue
int APPS_calc(uint16_t *torque){
    uint16_t fault, t_req = 0;

    float apps1 = REMAP0_1(ADC_Vars.APPS1, APPS1_MIN, APPS1_MAX),
          apps2 = REMAP0_1(ADC_Vars.APPS2, APPS2_MIN, APPS2_MAX),
          c_app = (apps1 + apps2) * 0.5f;

    if(apps1 < 0.0f || apps2 < 0.0f || apps2 > 1.0f || apps1 > 1.0f)
        fault = 1;
    else if (FABS(apps1 - apps2) > 0.1f)
        fault = 1;
    else if (c_app > 0.25f && ADC_Vars.FBPS > BRAKES_THREASHOLD)
        fault = 1;
    else
        t_req = REMAPm_M(c_app, MIN_TORQUE_REQ, MAX_TORQUE_REQ);

    GPIOB->ODR &= ~fault << 7;
    GPIOB->ODR |= fault << 7;

    *torque = t_req;
    return fault;
}

void send_CAN(uint16_t id, uint8_t length, uint8_t* data){
    if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        CAN->sTxMailBox[0].TDTR = length;
        for(int i = 0; i < length && i < 4; i++)
            CAN->sTxMailBox[0].TDLR = data[i]   << i * 8;
        for(int i = 0; i < length - 4; i++)
            CAN->sTxMailBox[0].TDHR = data[i+4] << i * 8;
        CAN->sTxMailBox[0].TIR = (uint32_t)((id << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ);
    }
}

void process_can(CAN_msg cm){
    switch (cm.id){
        case MC_CANID_HIGHSPEEDMESSAGE:
            car_state.hsmessage.bits = cm.data;
        break;
    }
}