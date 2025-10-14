/* Implements CAN features for the stm32h5 */

#ifndef _CAN_H_
#define _CAN_H_

void CAN_Init();

void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data);

#endif
