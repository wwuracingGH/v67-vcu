/*
 * Nicole Swierstra, Renee Anderson
 * Viking 67 main logic
 *  
 * State machine logic:
 * There are 4 main states:
 * Idle
 * Ready to drive
 * MC-init
 * MC-reset
 * 
 * Idle is the state the VCU defaults to upon reset
 * Ready to Drive is the state where the motor controller is energized and initialized
 * 
 */

#include <stdint.h>
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"
#include "llrttsos.h"
#include "modules/gpio.h"
#include "modules/logging.h"
#include "modules/flash.h"
#include "modules/control.h"
#include "modules/can.h"
#include "vendor/printf/printf.h"
#include "canDefinitions.h"
_RTOS_IMPLEMENTATION_

#define INPUT_BUTTONID_RTD 0


GPIO_TypeDef *  buttonPorts[] = { GPIOB };
uint8_t         buttonPins[]  = {    12 };
uint8_t         buttonAcLow[] = {     1 };
const uint8_t   buttonNum     = 1;

/*
 * WHICH PERIPHERALS ARE WHAT & CLOCK SPEED
 */
#define SYS_CLOCK 250000000UL
#define CTRL_CAN FDCAN1
#define DATA_CAN FDCAN2

#define LED_COLOR_IDLE      0x00FFFFFF /* light blue */
#define LED_COLOR_RTD       0x00FF00FF /* green */
#define LED_COLOR_FAULT     0xFF0000FF /* red */
#define LED_COLOR_WAITING   0xFFFFFFFF /* white */
#define LED_COLOR_SPECIAL   0x0000FFFF /* blue */
#define LED_COLOR_LEDOFF    0x00000000 /* black */

/*
 * EXTRA BEHAVIOR - 0 or 1 TODO: put this in flash - not a compiler target
 */
#define TRACTION_CONTROL            0 /* does nothing rn */
#define REGENBRAKING_ENABLED        0 /* does nothing rn */
#define MC_WATCHDOG_ENABLED         0 /* drops out of ready to drive upon a fault */
#define MC_ON_BABYSITTING           0 /* enables waiting for motor controller to get enabled */
#define MC_RESET_LOOP               0 /* enables reset loop */
#define CAN_WATCHDOG                0 /* resets can peripheral if no messages after 2 seconds - bad idea on test harness */
#define IGNORE_BRAKES               1 /* ignores brakes when checking for rtd and plaus */

/*
 * EXTRA BEHAVIOR CONFIG
 */
#define CAN_WATCHDOG_TIME        5000 /* 5000 ms before reset */


/*
 * RESETABLE FAULTS
 *
 * these are the faults that go into reset state instead of idle
 */
#define MC_RESET_BITMASK    ( \
                            MC_PFAULT_PRECHARGE_FAILURE | \
                            MC_PFAULT_PRECHARGE_TIMEOUT | \
                            MC_PFAULT_DC_BUS_VOLT_LOW   | \
                            \
                            MC_RFAULT_CAN_COMMAND_LOST  | \
                            MC_RFAULT_RESOLVER_DISCONNECTED \
                            )

/* Car State */
struct {
    int state_idle;
    int state_mcinit;
    int state_rtd;
    int state_reset;

    volatile uint16_t fault_counter;
    uint16_t plausibility_latch;
    uint16_t last_valid_tr;
    uint16_t last_torque_bounds;
    
    ControlReq_t          last_ctrl_vec;
    VehicleDynamicState_t last_state_vec;

    ADC_Block_t         adc_dat;
    MC_HighSpeed        mc_hsmsg;
    MC_FaultCodes       mc_faults; 
    MC_InternalStates   mc_istates;
    MC_VoltageInfo      mc_vinfo;
    DL_WheelSpeed       dl_wheelspeed;
    DL_CarAcceleration  dl_accel;

    uint32_t last_can_timestamp1; /* any message, fdcan1 */
    uint32_t last_can_timestamp2; /* any message, fdcan2 */
   
    /* more timestamps */
    uint32_t mc_hsmsg_timestamp;
    uint32_t mc_faults_timestamp;
    uint32_t mc_istates_timestamp;
    uint32_t mc_vinfo_timestamp;
    uint32_t dl_wheelspeed_timestamp;
    uint32_t dl_accel_timestamp;
} car_state;

void clock_init();

void MC_sendCommand();
void MC_sendStop();
int MC_faultedR() { return (MC_RESET_BITMASK & car_state.mc_faults.runtimeErrors); }
int MC_faulted()  { return car_state.mc_faults.postErrors || car_state.mc_faults.runtimeErrors; }

void Shared_processCAN();
void Shared_control();
void Shared_diagnostics();
void Shared_CANWatchdog();

void RTD_input();
void Idle_input();
void Reset_input();

void Reset_start();
void RTD_start();
void Idle_start();

void MCInit_loop();
void MCInit_start();

void BUZZERON()  { GPIOB->ODR |= GPIO_ODR_OD10;  }
void BUZZEROFF() { GPIOB->ODR &= ~GPIO_ODR_OD10; }

void msgCallback(uint8_t bus, uint32_t id, uint8_t dlc, uint32_t* data);
void memcpy_32(uint32_t* dest, uint32_t* src, uint32_t byte_size);
void systick_handler() { RTOS_Update(); }

const int control_period        = 200;
const int mc_command_period     =   5;
const int process_can_period    =   1;
const int diagnostics_period    = 250;
const int input_period          =  50;
const int mc_watchdog_period    = 999;

CarParameters_t* car_params;
ADC_Mult_t apps_mult = { 0 };

MC_Command          command_msg         = {   0, 0, 1, 0, 0, 0, 0, 0};
MC_ParameterCommand reset_msg           = {  20, 1, 0, 0, 0};
MC_ParameterCommand enable_fastm_msg    = { 227, 1, 0, 0xFFFE, 0}; /* TODO: verify */
MC_ParameterCommand shutup_msg          = { 148, 1, 0, 0b0001110011100111, 0xFFFF};

int main(void) {
    clock_init();
    GPIO_init();
    RTOS_init();
    CAN_init();
    CTRL_ADCinit();

    logging_init();
    _enable_logging_all();

    LOGLN("STARTING STM");

    car_state.plausibility_latch = 0;
    car_state.last_valid_tr = 0;

    car_state.state_idle   = RTOS_addState(Idle_start, NULL);
    car_state.state_mcinit = RTOS_addState(MCInit_start, NULL);
    car_state.state_rtd    = RTOS_addState(RTD_start, NULL);
    car_state.state_reset  = RTOS_addState(Reset_start, NULL);

    RTOS_scheduleTask(RTOS_ALL_STATES, Shared_processCAN, process_can_period);
    RTOS_scheduleTask(RTOS_ALL_STATES, Shared_diagnostics, diagnostics_period);
    RTOS_scheduleTask(RTOS_ALL_STATES, Shared_CANWatchdog, mc_watchdog_period);

    RTOS_scheduleTask(car_state.state_rtd, MC_sendCommand, mc_command_period);

    RTOS_scheduleTask(car_state.state_idle, MC_sendStop, mc_command_period);
    RTOS_scheduleTask(car_state.state_mcinit, MC_sendStop, mc_command_period);
    RTOS_scheduleTask(car_state.state_reset, MC_sendStop, mc_command_period);

    RTOS_scheduleTask(car_state.state_rtd, Shared_control, control_period);
    RTOS_scheduleTask(car_state.state_idle, Shared_control, control_period);

    RTOS_scheduleTask(car_state.state_rtd, RTD_input, input_period);
    RTOS_scheduleTask(car_state.state_idle, Idle_input, input_period);
    RTOS_scheduleTask(car_state.state_reset, Reset_input, input_period);

    RTOS_scheduleTask(car_state.state_mcinit, MCInit_loop, mc_command_period);

    RTOS_switchState(car_state.state_idle);

    car_params = FLASH_getVals();
    apps_mult = CTRL_getADCMultiplers(&car_params->adc_bounds);

    LOG("Hello World!\n");
    LOGLN("CLOCK CHECK: %d", SysTick->LOAD + 1);

    RTOS_start_armeabi(SYS_CLOCK);
    for (;;) {
        RTOS_ExecuteTasks();
    }
}

/* ======= RTD Specific functionality ======= */
void RTD_start() {
    GPIO_setLED(LED_COLOR_RTD);
    BUZZERON();
    command_msg.inverterEnable = 1;
    RTOS_scheduleEvent(BUZZEROFF, 1500);
}

void RTD_input() {
    GPIO_Update(RTOS_getMainTick());
    if (GPIO_buttonReleased(INPUT_BUTTONID_RTD)) {
        GPIO_buttonConsume(INPUT_BUTTONID_RTD); /* action has been processed */

        GPIO_setLED(0x0000FFFF);
    } else if (GPIO_buttonHeldTime(INPUT_BUTTONID_RTD, RTOS_getMainTick()) > 1000) {
    	GPIO_buttonConsume(INPUT_BUTTONID_RTD); /* action has been processed */
    	RTOS_switchState(car_state.state_idle);
    }
}

void MC_sendCommand() {
    CAN_sendmessage(CTRL_CAN, MC_CANID_COMMAND, 8, (uint8_t*)&command_msg);
}

void MC_watchdog() {
    if(!MC_WATCHDOG_ENABLED) return;
    
    if (MC_faultedR() && MC_RESET_LOOP)
        RTOS_switchState(car_state.state_reset);
    else if (MC_faulted()){
        RTOS_switchState(car_state.state_idle);
    }
}

/* ======= Idle Specific Functionality ======== */
void Idle_start() {
    GPIO_setLED(LED_COLOR_IDLE);
    command_msg.inverterEnable = 0;
    MC_sendCommand();
}

void Idle_input() {
	GPIO_Update(RTOS_getMainTick());

    if (MC_faulted()) {
        GPIO_setLED(LED_COLOR_FAULT);
    } else {
        GPIO_setLED(LED_COLOR_IDLE);
    }

    /* check that it is in a state of hard braking before getting into rtd */
    int braking = IGNORE_BRAKES || (car_state.last_ctrl_vec.brake_pressure > car_params->params.hard_braking);

    if (braking && GPIO_buttonReleased(INPUT_BUTTONID_RTD) && car_state.last_valid_tr < 10) {
        GPIO_buttonConsume(INPUT_BUTTONID_RTD);

        RTOS_switchState(car_state.state_mcinit);
    }
}

/* ======= Reset Specific Functionality ======== */
void Reset_start() {
    GPIO_setLED(LED_COLOR_FAULT);
}

void Reset_input() {
	GPIO_Update(RTOS_getMainTick());

    if (!MC_RESET_LOOP) 
        RTOS_switchState(car_state.state_idle);

    if (!MC_faultedR() && MC_faulted()) {
        RTOS_switchState(car_state.state_idle);
    }

    /* if pressed, cancel reset sequence */
    if (GPIO_buttonReleased(INPUT_BUTTONID_RTD)) {
        GPIO_buttonConsume(INPUT_BUTTONID_RTD);

        RTOS_switchState(car_state.state_idle);
    }
}

void MC_Reset(){
    CAN_sendmessage(CTRL_CAN, MC_CANID_PARAMCOM, 8, (uint8_t*)&reset_msg);
}

/* ======= MC Init Specific Functionality ======= */
void MCInit_start() {
    GPIO_setLED(LED_COLOR_WAITING);
    command_msg.inverterEnable = 1;
    MC_sendCommand();
}

void MCInit_loop() {
    if (car_state.mc_istates.vsmState == 7)
        RTOS_switchState(MC_RESET_LOOP ? car_state.state_reset : car_state.state_idle);
    if (car_state.mc_istates.vsmState == 6 || !MC_ON_BABYSITTING)
        RTOS_switchState(car_state.state_rtd);
}

/* ====== Shared functionality ====== */
void Shared_diagnostics() {
    CAN_sendmessage(DATA_CAN, VCU_CANID_APPS_RAW, 8, (uint8_t*)&car_state.adc_dat);
    CAN_sendmessage(DATA_CAN, VCU_CANID_BPS_RAW, 4, (uint8_t*)&car_state.adc_dat.FBPS);
    CAN_sendmessage(DATA_CAN, VCU_CANID_CTRL_VEC, 8, (uint8_t*)&car_state.last_ctrl_vec);
    VCU_VCUState st = { rtos_scheduler.state, car_state.fault_counter, car_state.plausibility_latch, car_state.last_valid_tr };

    CAN_sendmessage(DATA_CAN, VCU_CANID_VCU_STATE, 4, (uint8_t*)&st);
}

void Shared_processCAN() {
    CAN_Message_t * msg;
    uint32_t tick = RTOS_getMainTick();
    while((msg = CAN_getFirstMsg()) != 0) {
        uint32_t l = msg->dlc;
        if (msg->bus_id == 0) 
            car_state.last_can_timestamp1 = tick; 
        if (msg->bus_id == 1)
            car_state.last_can_timestamp2 = tick; 

        switch (msg->id) {
        case MC_CANID_HIGHSPEEDMESSAGE:
            memcpy_32((uint32_t*)&car_state.mc_hsmsg, msg->data, 2);
            car_state.mc_hsmsg_timestamp = tick;
            break;
        case DL_CANID_ACCEL:
            memcpy_32((uint32_t*)&car_state.dl_accel, msg->data, 2);
            car_state.dl_accel_timestamp = tick;
            break;
        case DL_CANID_WHEELSPEED:
            memcpy_32((uint32_t*)&car_state.dl_wheelspeed, msg->data, 2);
            car_state.dl_wheelspeed_timestamp = tick;
            break;
        case MC_CANID_FAULTCODES:
            memcpy_32((uint32_t*)&car_state.mc_faults, msg->data, 2);
            car_state.mc_faults_timestamp = tick;
            break;
        case MC_CANID_VOLTAGEINFO:
            memcpy_32((uint32_t*)&car_state.mc_vinfo, msg->data, 2);
            car_state.mc_vinfo_timestamp = tick;
            break;
        case MC_CANID_INTERNALSTATES:
            memcpy_32((uint32_t*)&car_state.mc_istates, msg->data, 2);
            car_state.mc_istates_timestamp = tick;
            break;
        /* TODO: Reprogram/Reveal */
        case VCU_CANID_PARAM_CHANGE:
        	VCU_ParamSet* ps = (VCU_ParamSet*)msg->data;
        	FLASH_storeVal(ps->id, ps->setValue, ps->write);
        	apps_mult = CTRL_getADCMultiplers(&car_params->adc_bounds);
        	break;
        case VCU_CANID_PARAM_REQUEST:
        	VCU_ParamReq* rq = (VCU_ParamSet*)msg->data;
        	VCU_ParamReveal rv = { FLASH_getVal(rq->id), rq->id };
        	CAN_sendmessage(DATA_CAN, VCU_CANID_PARAM_REVEAL, 6, (uint8_t*)&rv);
            break;
        default:
            break;
        }
    }
}

void Shared_CANWatchdog() {
    if (!CAN_WATCHDOG) return;

    uint32_t tick = RTOS_getMainTick();
    uint32_t can1d = tick - car_state.last_can_timestamp1;
    uint32_t can2d = tick - car_state.last_can_timestamp2;

    if (can1d > CAN_WATCHDOG_TIME || can2d > CAN_WATCHDOG_TIME)
        CAN_reset();
}

void Shared_control() {
    const int fault_ignore     = 20;
    const int fault_cutoff     = 80;
    const int fault_multiplier = 65536 / (fault_cutoff - fault_ignore);

    ADC_Block_t bl = CTRL_condense();
    car_state.adc_dat = bl;
    LOG("%d %d %d %d %d %d\n", bl.APPS1, bl.APPS2, bl.APPS3, bl.APPS4, bl.FBPS, bl.RBPS);

    //int max_torque = car_params->params.max_torque;
    int max_torque = 10;
    ControlReq_t tr = CTRL_getCommand(&apps_mult, &car_params->params, max_torque);

    /* the silly */
    if (IGNORE_BRAKES) tr.flags &= ~(APPS_FAULT_PLAUS | APPS_FAULT_BSE);

    if ((tr.flags & APPS_FAULT_PLAUS)) {
        car_state.plausibility_latch = 1;
    } else if (car_state.plausibility_latch) {
        if (tr.torque < (car_params->params.max_torque / 10)) {
            car_state.plausibility_latch = 0;
        }
    }

    if (!((tr.flags & 0xFF00) || car_state.plausibility_latch)) {
        car_state.fault_counter = 0;
        car_state.last_valid_tr = tr.torque;
        command_msg.torqueCommand = tr.torque;
    } else {
    	if (car_state.fault_counter < 100)
    		car_state.fault_counter += control_period;

        if (car_state.fault_counter < fault_ignore) {
            command_msg.torqueCommand = car_state.last_valid_tr;
        } else if (car_state.fault_counter < fault_cutoff) {         
            int j = fault_multiplier * (fault_cutoff - car_state.fault_counter); /* p15 fixed point (1092 ~= 2^16 / 60) */
            command_msg.torqueCommand = (j * car_state.last_valid_tr) >> 16UL;
        } else {
            command_msg.torqueCommand = 0;
        }
    }

    LOG("%d %d %d %d\n", command_msg.torqueCommand, car_state.fault_counter, tr.torque, tr.flags);
}

/* this is a bad idea I think */
void MC_sendStop() {
    int ltr = command_msg.torqueCommand;
    command_msg.torqueCommand = 0;
    MC_sendCommand();
    command_msg.torqueCommand = ltr;
}

/* ======= Various util functions ======= */
void clock_init() { /* turns up the speed to 250mhz */
    /* use voltage scaling profile 0 */
    PWR->VOSCR |= (0b11 << PWR_VOSCR_VOS_Pos);
    /* change the flash access latency */
    FLASH->ACR |= 5 << FLASH_ACR_LATENCY_Pos;
    /* change the flash delay */
    FLASH->ACR |= 2 << FLASH_ACR_WRHIGHFREQ_Pos;

    /* prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTEN;

    /* hsi clock /= 8 */
    RCC->CR |= (3 << RCC_CR_HSIDIV_Pos);

    /* enable pll1, divide incoming by 2 */
    RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1PEN | RCC_PLL1CFGR_PLL1QEN | (2 << RCC_PLL1CFGR_PLL1M_Pos);
    /* set the pll1 src to hsi clock, input freq is 2 - 4 */
    RCC->PLL1CFGR |= (1 << RCC_PLL1CFGR_PLL1SRC_Pos) | (1 << RCC_PLL1CFGR_PLL1RGE_Pos);

    /* set clock speed to 250mhz */
    RCC->PLL1DIVR &= ~(RCC_PLL1DIVR_PLL1N_Msk | RCC_PLL1DIVR_PLL1Q_Msk);
    RCC->PLL1DIVR |= ((((SYS_CLOCK/1000000UL) >> 1) - 1) << RCC_PLL1DIVR_PLL1N_Pos);
    RCC->PLL1DIVR |= (3 << RCC_PLL1DIVR_PLL1Q_Pos); /* PLL1_Q divide by 4 */

    /* Enables PLL1 and locks values */
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY));

    /* sets system clock source as PLL1 */
    RCC->CFGR1 |= 0b11 << RCC_CFGR1_SW_Pos;
    while (!(RCC->CFGR1 & (0b11 << RCC_CFGR1_SWS_Pos)));

    /* sets CAN source as PLL1 Q */
    RCC->CCIPR5 |= 0b01 << RCC_CCIPR5_FDCANSEL_Pos;
}
