#pragma once
#include <stdint.h>

/*
 * DEFINITIONS for motor controller usage as defined in this pdf:
 * https://app.box.com/s/vf9259qlaadhzxqiqrt5cco8xpsn84hk/file/27334613044
 */ 

#define CANSTRUCT struct __attribute__((packed, scalar_storage_order("little-endian")))

typedef int16_t     MC_Temperature;
typedef int16_t     MC_LowVoltage;
typedef int16_t     MC_Torque;
typedef int16_t     MC_HighVoltage;
typedef int16_t     MC_Current;
typedef int16_t     MC_Angle;
typedef int16_t     MC_AngularVelocity;
typedef uint8_t     MC_Boolean;
typedef int16_t     MC_Frequency;
typedef int16_t     MC_Time16;
typedef int32_t     MC_Time32;
typedef int16_t     MC_Flux;
typedef uint16_t    MC_ProportionalGain;
typedef uint16_t    MC_IntegralGain;
typedef uint16_t    MC_DerivativeGain;
typedef uint16_t    MC_LowPassFilterGain;
typedef uint16_t    MC_Pressure;
typedef int16_t     MC_Int;
typedef uint16_t    MC_UnsignedInt;
typedef uint8_t     MC_Byte;

#define MC_CANID_TEMPERATURE1              0x0A0
#define MC_CANID_TEMPERATURE2              0x0A1
#define MC_CANID_TEMPERATURE3              0x0A2
#define MC_CANID_ANALOGINVOLTAGES          0x0A3
#define MC_CANID_DIGITALINPUT              0x0A4
#define MC_CANID_MOTORPOSITION             0x0A5
#define MC_CANID_CURRENTINFO               0x0A6
#define MC_CANID_VOLTAGEINFO               0x0A7
#define MC_CANID_FLUXINFO                  0x0A8
#define MC_CANID_INTERNALVOLTAGES          0x0A9          
#define MC_CANID_INTERNALSTATES            0x0AA
#define MC_CANID_FAULTCODES                0x0AB
#define MC_CANID_TORQUETIMER               0x0AC
#define MC_CANID_MIANDFLUXWEAKENINGINFO    0x0AD
#define MC_CANID_FIRMWAREINFO              0x0AE
#define MC_CANID_DIAGNOSTIC                0x0AF

#define MC_CANID_HIGHSPEEDMESSAGE          0x0B0

#define MC_CANID_COMMAND                   0x0C0
#define MC_CANID_PARAMCOM                  0x0C1
#define MC_CANID_PARAMREQ                  0x0C2

#define EightByteEndianSwap(data)   \
( (((data) >> 56) & 0x00000000000000FF) | (((data) >> 40) & 0x000000000000FF00) | \
  (((data) >> 24) & 0x0000000000FF0000) | (((data) >>  8) & 0x00000000FF000000) | \
  (((data) <<  8) & 0x000000FF00000000) | (((data) << 24) & 0x0000FF0000000000) | \
  (((data) << 40) & 0x00FF000000000000) | (((data) << 56) & 0xFF00000000000000) ) 

const char* MC_Post_Errors[32] = {
    //0
    "Hardware Gate/Desaturation Fault",
    "Hardware Overcurrent Fault",
    "Accelerator Shorted",
    "Accelerator Open",
    "Current Sensor Low",
    "Current Sensor High",
    "Module Temperature Low",
    "Module Temperature High",
    //1
    "Control PCB Temperature Low",
    "Control PCB Temperature High",
    "GateDrive PCB Temperature Low",
    "GateDrive PCB Temperature High",
    "5V Sense Voltage Low",
    "5V Sense Voltage High",
    "12V Sense Voltage Low",
    "12V Sense Voltage High",
    //2
    "2.5V Sense Voltage Low",
    "2.5V Sense Voltage High",
    "1.5V Sense Voltage Low",
    "1.5V Sense Voltage High",
    "DC Bus Voltage High",
    "DC Bus Voltage Low",
    "Pre-Charge Timeout",
    "Pre-Charge Voltage Failure",
    //3
    "EEPROM Checksum Invalid",
    "EEPROM Data Out Of Range",
    "EEPROM Update Required",
    "Hardware DC Bus Over-Voltage during Initalization",
    "",
    "",
    "Brake Shorted",
    "Brake Open"
};

const char* MC_Run_Errors[32] = {
    //4
    "Motor Overspeed Fault",
    "Over-Current Fault",
    "Over-Voltage Fault",
    "Inverter Over-Temperature Fault",
    "Accelerator Input Shorted Fault",
    "Accelerator Input Open Fault",
    "Direction Command Fault",
    "Inverter Response Timeout Fault",
    //5
    "Hardware Gate Desaturation Fault",
    "Hardware Over-Current Fault",
    "Under-Voltage Fault",
    "CAN Command Message Lost Fault",
    "Motor Over-Temperature Fault",
    "",
    "",
    "",
    //6
    "Brake Input Shorted Fault",
    "Brake Input Open Fault",
    "Module A Over-Temperature Fault",
    "Module B Over-Temperature Fault",
    "Module C Over-Temperature Fault",
    "PCB Over-Temperature Fault",
    "Gate Drive Board 1 Over-Temperature Fault",
    "Gate Drive Board 2 Over-Temperature Fault",
    //7
    "Gate Drive Board 3 Over-temperature Fault",
    "Current Sensor Fault",
    "",
    "",
    "",
    "",
    "Resolver Not Connected",
    ""
};

CANSTRUCT MC_Temperature1 {
    MC_Temperature      moduleATemp;
    MC_Temperature      moduleBTemp;
    MC_Temperature      moduleCTemp;
    MC_Temperature      gateDriverBoardTemp;
};

CANSTRUCT MC_Temperature2 {
    MC_Temperature      controlBoardTemp;
    MC_Temperature      rtdInput1Temp;
    MC_Temperature      rtdInput2Temp;
    MC_Temperature      rtdInput3Temp;
};

CANSTRUCT MC_Temperature3 {
    MC_Temperature      rtdInput4Temp;
    MC_Temperature      rtdInput5Temp;
    MC_Temperature      motorTemp;
    MC_Torque           torqueShudder;
};

CANSTRUCT MC_AnalogInVoltages {
    MC_LowVoltage       AnalogIn1;
    MC_LowVoltage       AnalogIn2;
    MC_LowVoltage       AnalogIn3;
    MC_LowVoltage       AnalogIn4;
};

CANSTRUCT MC_DigitalInput {
    MC_Boolean          digitalInput1;
    MC_Boolean          digitalInput2;
    MC_Boolean          digitalInput3;
    MC_Boolean          digitalInput4;
    MC_Boolean          digitalInput5;
    MC_Boolean          digitalInput6;
    MC_Boolean          digitalInput7;
    MC_Boolean          digitalInput8;
};

CANSTRUCT MC_MotorPosition {
    MC_Angle            motorAngle;
    MC_AngularVelocity  motorSpeed;
    MC_Frequency        electricalOutputFrequency;
    MC_Angle            deltaResolverFiltered;
}; 

CANSTRUCT MC_CurrentInfo {
    MC_Current          phaseACurrent;
    MC_Current          phaseBCurrent;
    MC_Current          phaseCCurrent;
    MC_Current          dcBusCurrent;
};

CANSTRUCT MC_VoltageInfo {
    MC_Current          dcBusVoltage;
    MC_Current          outputVoltage;
    MC_Current          VABVoltage;
    MC_Current          VBCVoltage;
};   

CANSTRUCT MC_FluxInfo {
    MC_Flux             fluxCommand;
    MC_Flux             fluxFeedback;
    MC_Current          ldFeedback;
    MC_Current          lqFeedback;
};

CANSTRUCT MC_InternalVoltages {
    MC_LowVoltage       ref1Point5V;
    MC_LowVoltage       ref2Point5V;
    MC_LowVoltage       ref5V;
    MC_LowVoltage       ref12V;
};

CANSTRUCT MC_InternalStates {
    MC_Byte             vsmState;
    MC_Byte             _RESERVED1;
    MC_Byte             inverterState;
    MC_Byte             relayState;
    uint8_t             inverterRunMode             : 1;
    uint8_t             selfSensingAssistEnable     : 1;
    uint8_t             _RESERVED2                  : 3;
    uint8_t             inverterActiveDischargeMode : 3;
    MC_Boolean          inverterCommandMode         : 4;
    uint8_t             rollingCounterValue         : 4;
    MC_UnsignedInt      whateverElse;
};

CANSTRUCT MC_FaultCodes {
    uint32_t            postErrors;
    uint32_t            runtimeErrors;
};

CANSTRUCT MC_TorqueTimer {
    MC_Torque           commandedTorque;
    MC_Torque           torqueFeedback;
    MC_Time32           powerOnTimer;
};

CANSTRUCT MC_MIandFluxWeakeningInfo {
    MC_UnsignedInt      modulationIndex;
    MC_Current          fluxWeakening;
    MC_Current          ldCommand;
    MC_Current          lqCommand;
};

CANSTRUCT MC_FirmwareInfo {
    MC_UnsignedInt      version;
    MC_UnsignedInt      softwareVersion;
    MC_UnsignedInt      dateCodeMMDD;
    MC_UnsignedInt      dateCodeYYYY;
}; 

CANSTRUCT MC_Diagnostic {
    uint32_t a, b;
}; 

CANSTRUCT MC_HighSpeed {
    MC_Torque           torqueCommand;
    MC_Torque           torqueFeedback;
    MC_AngularVelocity  motorSpeed;
    MC_HighVoltage      dcBusVoltage;
};

CANSTRUCT MC_Command {
    MC_Torque           torqueCommand;
    MC_AngularVelocity  speedCommand;
    MC_Boolean          directionCommand;
    MC_Byte             controlByte;
    MC_Torque           torqueLimit;
};