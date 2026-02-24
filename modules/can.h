/*
 * Nicole Swierstra, Gus Rindal 
 *
 * CAN Driver for the stm32h5 -> message send and recieve capabilities
 * uses a callback structure for the recieve
 */

#ifndef _MODULES_CAN_H_
#define _MODULES_CAN_H_
#include "stm32h5xxCANSRAM.h"
#include "stm32h5xx.h"

#define CAN1_BAUD 250000UL
#define CAN2_BAUD 1000000UL

#define CAN1_BAUD_PRES ((1250000UL - CAN1_BAUD) / 10000UL)
#define CAN2_BAUD_PRES ((1250000UL - CAN2_BAUD) / 10000UL)

void CAN_init();
void CAN_reset();

typedef struct {
    uint32_t id;
    uint16_t dlc;
    uint16_t bus_id;
    uint32_t data[2];
} CAN_Message_t;

void memcpy_32(uint32_t* dest, uint32_t* src, uint32_t l);

void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data);
int CAN_rxCount();
CAN_Message_t* CAN_getFirstMsg();

int CAN_bytesFromDLC(int dlc);

#endif
