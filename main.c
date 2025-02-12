/**
 * NICOLE SWIERSTRA
 * 
 * VIKING 66 VCU CODE
 *
 * This is the code for the vcu of the wwu racing viking 66 car meant for competition in 2024
 *
 * documentation can be found somewhere in the q drive and maybe in the readme if I ever get to it.
 */

#include <stdint.h>
#ifndef STM32F042x6
#define STM32F042x6
#endif
#include "vendor/CMSIS/Device/ST/STM32F0/Include/stm32f0xx.h"
#include "canDefinitions.h"
#include "vendor/qfplib/qfplib.h"
#include "rtos.h"

/*
 * ADC PARAMETERS 
 */
#define ROLLING_ADC_FR_POW 5
#define ROLLING_ADC_FRAMES (1 << ROLLING_ADC_FR_POW) 
#define ROLLING_ADC_VALS  (ROLLING_ADC_FRAMES * 4)

/*
 * EXTRA BEHAVIOR - 0 or 1
 */
#define TRACTIONCONTROL     0 /* does nothing rn */
#define REGENBRAKING        0 /* does nothing rn */
#define MC_RESET_BITMASK    0x00000400

/*
 * HELPER FUNCTIONS
 */
#define CONSTINV(n)             (1.0f / (float)(n))
#define REMAP0_1(n, min, max)   ((float)(n - min) * CONSTINV(max - min))
#define REMAPm_M(n, min, max)   ((n) * (max - min) + (min))
#define FABS(x)                 ((x) > 0.0f ? (x) : -(x))

/**
 * I HAVE NO IDEA HOW THE MOSFET WORKS
 */

#define BUZZERON()              (GPIOB->ODR |= GPIO_ODR_5)
#define BUZZEROFF()             (GPIOB->ODR &= ~GPIO_ODR_5)

/*
 * APPS CALC PARAMETERS
 *
 * These are settings that are both compiled and written to by the  
 */

#define COMPILED_MIN_REGEN_SPEED     0
#define COMPILED_MIN_TORQUE_REQ      0
#define COMPILED_MAX_TORQUE_REQ      500
#define COMPILED_BRAKES_THREASHOLD   450

#define COMPILED_APPS1_MIN 1130
#define COMPILED_APPS2_MIN 1823
#define COMPILED_APPS1_MAX 1996
#define COMPILED_APPS2_MAX 2700

typedef struct  {
    uint16_t APPS1_MIN;
    uint16_t APPS1_MAX;
    uint16_t APPS2_MIN;
    uint16_t APPS2_MAX;
} _appsCalibrationValues; 

typedef struct  {
    uint16_t MAX_TORQUE_REQ;
    uint16_t MIN_TORQUE_REQ;
    uint16_t MIN_REGEN_SPEED;
    uint16_t BRAKES_THREASHOLD;
} _appsCalcParameters;

typedef struct {
    uint32_t size;
    _appsCalibrationValues calibration;
    _appsCalcParameters    calculation;
} _settings;

static const volatile _settings config __attribute__((section(".config"))) = {
    sizeof(config),
    {
        COMPILED_APPS1_MIN, 
        COMPILED_APPS1_MAX, 
        COMPILED_APPS2_MIN, 
        COMPILED_APPS2_MAX
    },
    {
        COMPILED_MAX_TORQUE_REQ,
        COMPILED_MIN_TORQUE_REQ,
        COMPILED_MIN_REGEN_SPEED,
        COMPILED_BRAKES_THREASHOLD,
    }
};

#define APPS1_MIN config.calibration.APPS1_MIN
#define APPS1_MAX config.calibration.APPS1_MAX
#define APPS2_MIN config.calibration.APPS2_MIN
#define APPS2_MAX config.calibration.APPS2_MAX

#define MIN_TORQUE_REQ      config.calculation.MIN_TORQUE_REQ
#define MAX_TORQUE_REQ      config.calculation.MAX_TORQUE_REQ
#define MIN_REGEN_SPEED     config.calculation.MIN_REGEN_SPEED
#define BRAKES_THREASHOLD   config.calculation.BRAKES_THREASHOLD

/*
 * APPS DEADZONE
 */
const float SENSOR_MIN = -0.15f;
const float SENSOR_MAX =  1.15f;

/* 
 * RTOS FUNCTION PERIODS - in MS
 */
const uint16_t controlPeriod = 5,
               inputPeriod = 50,
               recievePeriod = 20,
               diagPeriod = 100,
               canWDPeriod = 2500;

/* 
 * helper 
 */
const uint8_t MCResetMaxAttempts = 5;

struct __attribute__((packed)) {
    volatile uint16_t APPS2;
    volatile uint16_t RBPS;
    volatile uint16_t FBPS;
    volatile uint16_t APPS1;
} ADC_Vars;

uint16_t ADC_RollingValues[ROLLING_ADC_VALS];

struct {
    int state_idle, state_rtd;
    volatile uint16_t torque_req;
    uint16_t lastAPPSFault;
    uint8_t MCResetAttempts; /* number of resets until it stops */
    struct{
        uint16_t apps1, apps2, torque, fault;
    } APPSCalib;
    union {
        MC_HighSpeed hs;
        uint64_t bits;
    } hsmessage;
    union {
        MC_FaultCodes fc;
        uint64_t bits;
    } faults;
    union {
        MC_InternalStates is;
        uint64_t bits;
    } mcstate;
    union {
        MC_VoltageInfo vi;
        uint64_t bits;
    } voltageinfo;
    union {
        DL_WheelSpeed ws;
        uint64_t bits;
    } wheelspeed;
    union {
        DL_CarAcceleration ca;
        uint64_t bits;
    } acceleration;
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
void ADC_DMA_Init(uint32_t *dest, uint32_t size);
void GPIO_Init();
void CAN_Init();
void flash_Init();

void default_handler();
void Control();
void InputIdle();
void InputRTD();
void send_Diagnostics();
void CanReset();
void MCWatchdog();

void APPS_RollingSmooth();
int  APPS_calc(uint16_t*, uint16_t);
void send_CAN(uint16_t, uint8_t, uint8_t*);
void process_CAN(uint16_t id, uint8_t length, uint64_t data);
void recieve_CAN();
void RTD_start();
void Idle_start();

int areThereFaults();

void reprogram(_settings nc);
void reprogramAPPS(VCU_ReprogramApps ra);
void reprogramControl(VCU_ReprogramControl rc);

/* global stuff */
MC_Command canmsg = {0, 0, 1, 0, 0, 0, 0, 0};
MC_ParameterCommand resetMC = {20, 1, 0, 0, 0};
MC_ParameterCommand torqueLimitMsg = { 129, 1, 0, COMPILED_MAX_TORQUE_REQ, 0 };
MC_ParameterCommand fastMsg = {227, 1, 0, 0xFFFE, 0}; /* TODO: verify */
MC_ParameterCommand shutup = { 148, 1, 0, 0b0001110011100111, 0xFFFF};

kernel rtos_scheduler = {0, -1, {{0, 0, 0}}, 0, 0, {{0, 0, 0}}, 0, 0, {{0, 0, 0}}}; 

int main(){
    /* setup */
    clock_init();

    for(int i = 0; i < ROLLING_ADC_VALS; i++){
        ADC_RollingValues[i] = 0;
    }

    flash_Init();

    GPIO_Init(); /* must be called first */
 
    BUZZEROFF(); /* turns that buzzer the fuck off */
    
    RTOS_init();

    car_state.state_idle = RTOS_addState(Idle_start, 0);
    car_state.state_rtd  = RTOS_addState(RTD_start, 0);

    RTOS_scheduleTask(car_state.state_idle, send_Diagnostics, diagPeriod);
    RTOS_scheduleTask(car_state.state_idle, Control, controlPeriod);
    RTOS_scheduleTask(car_state.state_idle, InputIdle, inputPeriod);
    RTOS_scheduleTask(car_state.state_idle, recieve_CAN, recievePeriod);

    RTOS_scheduleTask(car_state.state_idle, MCWatchdog, 1000); /* Every one second */
    RTOS_scheduleTask(car_state.state_rtd,  MCWatchdog, 1000); /* Every one second */
    
    RTOS_scheduleTask(car_state.state_rtd, InputRTD, inputPeriod);
    RTOS_scheduleTask(car_state.state_rtd, Control, controlPeriod);
    RTOS_scheduleTask(car_state.state_rtd, send_Diagnostics, diagPeriod);
    RTOS_scheduleTask(car_state.state_rtd, recieve_CAN, recievePeriod);

    ADC_DMA_Init((uint32_t *)ADC_RollingValues, ROLLING_ADC_VALS);
    CAN_Init();

    /* TODO: shutup every time the MC yaps */
    send_CAN(MC_CANID_PARAMCOM, 8, (uint8_t *)&shutup);
    send_CAN(MC_CANID_PARAMCOM, 8, (uint8_t *)&torqueLimitMsg);

    RTOS_switchState(car_state.state_idle);

    SysTick_Config(48000); /* 48MHZ / 48000 = 1 tick every ms */
    __enable_irq(); /* enable interrupts */

    /* non rt program bits */
    for(;;){
        RTOS_ExecuteTasks();
    }
}

/* runs every 1 ms */
void systick_handler(){  
    RTOS_Update();
}

void flash_Init(){
    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
        FLASH->KEYR = (uint32_t)0x45670123;
        FLASH->KEYR = (uint32_t)0xCDEF89AB;
    }
}

void Control() {
    APPS_RollingSmooth();
    car_state.lastAPPSFault = APPS_calc((uint16_t *)&car_state.torque_req, car_state.lastAPPSFault);

    canmsg.torqueCommand = car_state.torque_req;
    send_CAN(MC_CANID_COMMAND, 8, (uint8_t*)&canmsg);    
}

void InputIdle(){
    if(!(GPIOB->IDR & GPIO_IDR_1)){
        if (((ADC_Vars.APPS1 <= APPS1_MIN) && (ADC_Vars.APPS2 <= APPS2_MIN))
            && !areThereFaults())
        {
            if (car_state.mcstate.is.inverterState != 4) return;
            
            RTOS_switchState(car_state.state_rtd);

            while(car_state.mcstate.is.inverterState == 5) recieve_CAN();

            if(car_state.mcstate.is.inverterState != 6) RTOS_switchState(car_state.state_idle);
        }
        else if (areThereFaults()) {
            MCWatchdog();
        }
    }
}

void InputRTD(){


}

void send_Diagnostics(){ 
    send_CAN(VCU_CANID_APPS_RAW, 8, (uint8_t *)&ADC_Vars.APPS2);
    send_CAN(VCU_CANID_CALIBRATION, 8, (uint8_t *)&car_state.APPSCalib.apps1);
    send_CAN(VCU_CANID_REPROGRAMAPPS, 8, (uint8_t *)&config.calibration);

    /* TODO: this but better */
    uint8_t statemsg[2] = {
        RTOS_inState(car_state.state_idle),
        RTOS_inState(car_state.state_rtd)
    };
    send_CAN(VCU_CANID_STATE, 2, statemsg);
}

void __turn_that_buzzer_the_fuck_off(){
    BUZZEROFF(); /* turns that buzzer the fuck off */
}

int areThereFaults(){
    return !!( car_state.faults.fc.runtimeErrors & MC_RESET_BITMASK);
}

void Idle_start(){
    canmsg.inverterEnable = 0;
    GPIOA->ODR |= 1 << 9;
}

void RTD_start(){
    BUZZERON(); /* buzzer */ 
    send_CAN(MC_CANID_COMMAND, 8, (uint8_t *)&canmsg);
    GPIOA->ODR |= 1 << 9;
    canmsg.inverterEnable = 1;
    RTOS_scheduleEvent(__turn_that_buzzer_the_fuck_off, 1500);
}

void MCWatchdog(){
    if(areThereFaults()){
        if(RTOS_inState(car_state.state_rtd)){
            RTOS_switchState(car_state.state_idle);
        }
        
        /* waits until discharge is complete */
        while(car_state.voltageinfo.vi.dcBusVoltage > 20); 

        send_CAN(MC_CANID_PARAMCOM, 8, (uint8_t*)&resetMC);
    } 
}

/* incredible code right here */
void reprogram(_settings newConfig) {
   
    uint32_t configptr = (uint32_t)&config; 
    
    /* 0x007C0008 */
    /* 0x00008000 */
    send_CAN(0x12, 4, (uint8_t *)&configptr);

    if (configptr < 0x08007C00 || configptr >= 0x08008000 - sizeof(config)) return;
    
    GPIOB->ODR &= ~(1 << 6);
    for(int i = 0; i < 10000; i++);
    GPIOB->ODR |= (1 << 6);

    FLASH->CR |= FLASH_CR_PER; /* enable page erase */
    FLASH->AR = 0x08007C00; /*  start at the 31st page */
    FLASH->CR |= FLASH_CR_STRT; /* start erasing */
    while ((FLASH->SR & FLASH_SR_BSY) != 0); /* wait until it's done */
    if ((FLASH->SR & FLASH_SR_EOP) != 0) /* clear the eop bit */
    {
        FLASH->SR = FLASH_SR_EOP;
    }
    
    FLASH->CR &= ~FLASH_CR_PER; /* Turn off page erase */


    FLASH->CR |= FLASH_CR_PG;
    for(uint32_t i = 0; i < sizeof(newConfig)/2; i++){
        ((uint16_t*)&config)[i] = ((uint16_t*)&newConfig)[i];
        while ((FLASH->SR & FLASH_SR_BSY) != 0);
    }

    FLASH->CR &= FLASH_CR_PG;

    NVIC_SystemReset();
     
}

void reprogramAPPS(VCU_ReprogramApps ra){
    _settings newConfig = {
        sizeof(newConfig),
        {
            ra.new_APPS1_MIN,
            ra.new_APPS1_MAX,
            ra.new_APPS2_MIN,
            ra.new_APPS2_MAX
        },
        {
            MAX_TORQUE_REQ,
            MIN_TORQUE_REQ,
            MIN_REGEN_SPEED,
            BRAKES_THREASHOLD
        }
    };

    reprogram(newConfig);
}

void reprogramControl(VCU_ReprogramControl rc){
    _settings newConfig = {
        sizeof(newConfig),
        {
            APPS1_MIN,
            APPS1_MAX,
            APPS2_MIN,
            APPS2_MAX
        },
        {
            rc.new_MaxTorqueReq,
            rc.new_MinRegenReq,
            rc.new_MinRegenSpeed,
            rc.new_BrakeThreashold
        }
    };

    reprogram(newConfig); 
}

void clock_init() /* turns on hsi48 and sets as system clock */
{
    /* wait one clock cycle before accessing flash memory @48MHZ */
    FLASH->ACR |= 0b001 << FLASH_ACR_LATENCY_Pos;

    /* Enables HSI48 oscillator */
    RCC->CR2  |= RCC_CR2_HSI48ON;
    while (!(RCC->CR2 & RCC_CR2_HSI48RDY));
    
    /* no peripheral prescaler div or hsi prescaler div */
    RCC->CFGR &= ~(0b111 << RCC_CFGR_PPRE_Pos);
    RCC->CFGR &= ~(0b1111 << RCC_CFGR_HPRE_Pos);

    /* sets system clock as HSI48 oscillator */
    RCC->CFGR |= 0b11 << RCC_CFGR_SW_Pos;
    while (!(RCC->CFGR & (0b11 << RCC_CFGR_SWS_Pos)));
}

void ADC_DMA_Init(uint32_t *dest, uint32_t size){
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 &= ~(uint32_t)0b011000; /* set 12 bit precision */
    
    ADC1->CFGR1 |= ADC_CFGR1_CONT; /* analog to digital converter to cont mode */
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; /* align bits to the right */

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG; /* enable dma & make cont */

    ADC1->SMPR |= 0b111;

    ADC1->CHSELR |= ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL5 
                 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL8; /*channels to scan */

    /* make sure the ADC's weird clock is on */
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
    ADC1->CFGR2 &= (~ADC_CFGR2_CKMODE);

    /* turn on interrupts for dma i think */
    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
        ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);

    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM; /* peripheral to memory */
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_TEIE; /* c */
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; /* set size of data to transfer */
    
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR)); /* sets source of dma transfer */
    DMA1_Channel1->CMAR = (uint32_t)dest; /* sets destination of dma transfer */
    DMA1_Channel1->CNDTR = size; /* sets size of dma transfer */
    
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* enables dma */

	ADC1->CR |= ADC_CR_ADSTART; /* starts adc */

    /* Turns on the actual interrupts */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); 
    NVIC_SetPriority(DMA1_Channel1_IRQn,0);
}

void CAN_Init (){
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;
    CAN->MCR |= CAN_MCR_INRQ; /* goes from normal mode into initialization mode */
    
    while (!(CAN->MSR & CAN_MSR_INAK));
   
    /* wakes it up */
    CAN->MCR &= ~CAN_MCR_SLEEP;
    while (CAN->MSR & CAN_MSR_SLAK);

    /* set bittiming - just read wikipedia if you don't know what that is */
    /* TODO: why is it still 1/2 of what it should be */
    CAN->BTR |= 23 << CAN_BTR_BRP_Pos | 1 << CAN_BTR_TS1_Pos | 0 << CAN_BTR_TS2_Pos;
    CAN->MCR &= ~CAN_MCR_INRQ; /* clears the initialization request and starts the actual can */
    
    while (CAN->MSR & CAN_MSR_INAK);

    /* blank filter - tells the can to read every message */
    CAN->FMR |= CAN_FMR_FINIT; 
    CAN->FA1R |= CAN_FA1R_FACT0;
    CAN->sFilterRegister[0].FR1 = 0; /* Its like a filter, but doesn't filter anything! */
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->FMR &=~ CAN_FMR_FINIT;
    CAN->IER |= CAN_IER_FMPIE0;
}
    
void GPIO_Init(){
    /* turn on gpio clocks */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

    GPIOA->MODER |= (MODE_OUTPUT    << GPIO_MODER_MODER0_Pos)   /* PORTA_GPIO   */
                 |  (MODE_ANALOG    << GPIO_MODER_MODER1_Pos)   /* APPS2        */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER3_Pos)   /* PORTA_GPIO   */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER4_Pos)   /* PORTA_GPIO   */
                 |  (MODE_ANALOG    << GPIO_MODER_MODER5_Pos)   /* RBPS         */
                 |  (MODE_ANALOG    << GPIO_MODER_MODER6_Pos)   /* FBPS         */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER9_Pos)   /* RTD_LIGHT    */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODER11_Pos)  /* CAN TX       */
                 |  (MODE_ALTFUNC   << GPIO_MODER_MODER12_Pos); /* CAN RX       */

    /* no pull up - pull down */
    GPIOA->PUPDR  = 0 | (0b10 << GPIO_PUPDR_PUPDR1_Pos);
    GPIOB->PUPDR  = 0;
    GPIOA->AFR[1] = (4 << GPIO_AFRH_AFSEL11_Pos) | (4 << GPIO_AFRH_AFSEL12_Pos); /* can AFR */

    GPIOB->MODER |= (MODE_ANALOG    << GPIO_MODER_MODER0_Pos)   /* APPS1        */
                 |  (MODE_INPUT     << GPIO_MODER_MODER1_Pos)   /* RTD_BUTTON   */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER3_Pos)   /* Nucleo user LED */
                 |  (MODE_INPUT     << GPIO_MODER_MODER4_Pos)   /* PORTB_GPIO   */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER5_Pos)   /* BUZZER       */
                 |  (MODE_OUTPUT    << GPIO_MODER_MODER6_Pos)   /* LED1         */
                 |  (MODE_INPUT     << GPIO_MODER_MODER7_Pos);  /* PORTB_GPIO   */
    
    GPIOB->PUPDR |= 0b01 << GPIO_PUPDR_PUPDR1_Pos; /* button is pulled up */
}

/* very fast average */
void APPS_RollingSmooth(){
    uint32_t APPS2 = 0,
             RBPS  = 0,
             FBPS  = 0,
             APPS1 = 0;
    for(int i = 0; i < ROLLING_ADC_VALS; i += 4){
        APPS2 += ADC_RollingValues[i + 0];
        RBPS  += ADC_RollingValues[i + 1];
        FBPS  += ADC_RollingValues[i + 2];
        APPS1 += ADC_RollingValues[i + 3];
    }

    ADC_Vars.APPS2 = APPS2 >> ROLLING_ADC_FR_POW;
    ADC_Vars.RBPS  = RBPS  >> ROLLING_ADC_FR_POW;
    ADC_Vars.FBPS  = FBPS  >> ROLLING_ADC_FR_POW;
    ADC_Vars.APPS1 = APPS1 >> ROLLING_ADC_FR_POW;
}

int GetTCMax(){
#if TRACTIONCONTROL == 1
    const float mass_KG = 240;
    const float rearAxel_Moment = 0.78f;
    const float gravity = 9.81f;
    const float cg_height = 22.86f;
    const float wheelbase = 1.54f;
    const float wheelbasediv = 1.0f/wheelbase;

    const float ellipseOblongConst = 1.2f; /*reciprocal of how weak the side friction is compared to the long friction */

    float rearAxel_downforce = ((2 * (wheelbase - rearAxel_Moment) * gravity) 
        + (cg_height * car_state.acceleration.ca.carAccel_X)) 
        * (0.5f * wheelbasediv)
        * mass_KG;
    float rearAxel_Force_Y = (wheelbase - rearAxel_Moment) * wheelbasediv * mass_KG 
        * car_state.acceleration.ca.carAccel_Y;

    float usable_ux = qfp_fsqrt(1 - (rearAxel_downforce * rearAxel_downforce)) * ellipseOblongConst;

    return 0;
#else
    return MAX_TORQUE_REQ; 
#endif
}

/* returns 1 if there's an issue */
int APPS_calc(uint16_t *torque, uint16_t lastFault){
    static uint8_t faultCounter = 0;
    const uint32_t maxFaultCount = 20; //(100 / controlPeriod) - 1; /* 100ms */

    uint16_t fault = 0, t_req = 0;
    
    const float apps1div = 1.0f / (APPS1_MAX - APPS1_MIN);
    const float apps2div = 1.0f / (APPS2_MAX - APPS2_MIN);

    float apps1 = ((float)ADC_Vars.APPS1 - APPS1_MIN) * apps1div,
          apps2 = ((float)ADC_Vars.APPS2 - APPS2_MIN) * apps2div; 

    if(apps1 < SENSOR_MIN || apps2 < SENSOR_MIN || apps1 > SENSOR_MAX || apps2 > SENSOR_MAX)
        fault = 1;
    else {
        apps1 = (apps1 > 1.0f) ? 1.0f : apps1;
        apps1 = (apps1 < 0.0f) ? 0.0f : apps1;
        apps2 = (apps2 > 1.0f) ? 1.0f : apps2;
        apps2 = (apps2 < 0.0f) ? 0.0f : apps2;
    }

    float c_app = (apps1 + apps2) * 0.5f;

    if      (lastFault == 3 && c_app > 0.05f)
        fault = 3;
    else if (fault == 1)
                 ;
    else if (FABS(apps1 - apps2) > 0.1f)
        fault = 2;
    else if (c_app > 0.25f && (ADC_Vars.FBPS > BRAKES_THREASHOLD))
        fault = 3;
    else
        faultCounter = 0;

    if (fault != 0) faultCounter++;

    car_state.APPSCalib.apps1  = (uint16_t)(apps1 * 1000);
    car_state.APPSCalib.apps2  = (uint16_t)(apps2 * 1000);
    car_state.APPSCalib.torque = (uint16_t)t_req;
    car_state.APPSCalib.fault  = (uint16_t)fault;

    if (faultCounter > maxFaultCount || RTOS_inState(car_state.state_idle)){
        t_req = 0; 
    }
    else {
        if (faultCounter > maxFaultCount - 10)
            t_req = car_state.torque_req - 240; /* gradual decrease to ensure no resolver faults */
        else 
            t_req = REMAPm_M(c_app, MIN_TORQUE_REQ, MAX_TORQUE_REQ);
    }
 

#if REGENBRAKING == 1
    /* TODO: REGEN? */
#endif 
    
    if(t_req > GetTCMax()) t_req = GetTCMax();

    *torque = t_req;
    return fault;
}

void send_CAN(uint16_t id, uint8_t length, uint8_t* data){
    /* find first empty mailbox */
    int j = (CAN->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;
    
    /* set dlc to length */
    CAN->sTxMailBox[j].TDTR = length;

    /* clears data high/low registers */
    CAN->sTxMailBox[j].TDLR = 0;
    CAN->sTxMailBox[j].TDHR = 0;
    
    /* writes to high/low registers */
    for(int i = 0; i < length && i < 4; i++) 
        CAN->sTxMailBox[j].TDLR |= ((data[i] & 0xFF) << i * 8);
    for(int i = 0; i < length - 4; i++)
        CAN->sTxMailBox[j].TDHR |= ((data[i+4] & 0xFF) << i * 8);
   
    /* writes id and queues message */
    CAN->sTxMailBox[j].TIR = (uint32_t)((id << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ);
}

void recieve_CAN(){
    /* while mailboxes aren't empty */
    while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) { 
        uint8_t  can_len    = CAN->sFIFOMailBox[0].RDTR & 0xF;
        uint64_t can_data   = CAN->sFIFOMailBox[0].RDLR 
            + ((uint64_t)CAN->sFIFOMailBox[0].RDHR << 32);
        uint16_t can_id     = CAN->sFIFOMailBox[0].RIR >> CAN_RI0R_STID_Pos;
        CAN->RF0R |= CAN_RF0R_RFOM0; /* release mailbox */

        process_CAN(can_id, can_len, can_data);
    }
}

void process_CAN(uint16_t id, uint8_t length, uint64_t data){

    (void)(length);

    switch (id){
        case MC_CANID_HIGHSPEEDMESSAGE:
            car_state.hsmessage.bits = data;
            break;
        case VCU_CANID_ACCEL:
            car_state.acceleration.bits = data;
            break;
        case VCU_CANID_WHEELSPEED:
            car_state.wheelspeed.bits = data;
            break;
        case MC_CANID_FAULTCODES:
            car_state.faults.bits = data;
            break;
        case MC_CANID_VOLTAGEINFO:
            car_state.voltageinfo.bits = data;
            break;
        case MC_CANID_INTERNALSTATES:
            car_state.mcstate.bits = data;
            break;
        case VCU_CANID_REPROGRAMAPPS:
            reprogramAPPS(*(VCU_ReprogramApps *)&data);
            break;
        case VCU_CANID_REPROGRAMCONTROL:
            reprogramControl(*(VCU_ReprogramControl *)&data);
            break;
        case VCU_CANID_REVEAL_VALS:
            send_CAN(VCU_CANID_APPS_VALS, 8, (uint8_t *)&config.calibration);
            send_CAN(VCU_CANID_CONTROL_VALS, 8, (uint8_t *)&config.calculation);
            break;
    }
}
