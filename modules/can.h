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
#define CAN2_BAUD 250000UL

#define CAN1_BAUD_PRES ((1250000UL - CAN1_BAUD) / 10000UL)
#define CAN2_BAUD_PRES ((1250000UL - CAN2_BAUD) / 10000UL)

void CAN_init();
void CAN_reset();

void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data);
void CAN_recieveMessages(void (*callback)(uint8_t bus, uint32_t id, uint8_t dlc, uint32_t* data));

int CAN_bytesFromDLC(int dlc);

#endif
