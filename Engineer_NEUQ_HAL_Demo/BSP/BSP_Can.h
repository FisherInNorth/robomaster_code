#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx.h"
#include "can.h"

#define CANRX_Mineral_Motor_F 		0x201
#define CANRX_Mineral_Motor_B 		0x202
#define CANRX_Mineral_Motor_R 		0x203
#define CANRX_Mineral_Motor_L 		0x204
#define CANRX_Flip_Motor_R 				0x205
#define CANRX_Flip_Motor_L 				0x206
#define CANRX_Handle_Motor				0x207


uint8_t Bsp_CanInit(void) ;   
void CAN1_Send_Mineral_Msg(uint8_t* msg);
void CAN1_Send_Flip_Msg(uint8_t* msg);    
uint8_t CAN1_Send_Handle_Msg(uint8_t* msg);



#endif

