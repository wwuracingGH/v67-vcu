/* Implements CAN features for the stm32h5 */

#ifndef _CAN_H_
#define _CAN_H_
#include "stm32h5xxCANSRAM.h"
#include "stm32h5xx.h"

void CAN_Init();

void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data);
void CAN_recieveMessages(void (*processMessage)(uint8_t bus, uint32_t id, uint8_t dlc, uint32_t* data));

#endif
