#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"
#include "BSP_Chassis.h"
#include "BSP_Rescue_card.h"
#include "BSP_Rescue_claw.h"






#define CAN_3508Motor1_ID    0x201
#define CAN_3508Motor2_ID    0x202
#define CAN_3508Motor3_ID    0x203
#define CAN_3508Motor4_ID    0x204
#define CAN_3508Motor5_ID    0x205
#define CAN_3508Motor6_ID    0x206
#define CAN_2006Motor7_ID    0x207
#define LOOP_BACK_ID         0x200






void bsp_can1_init(void);
void bsp_can2_init(void);
	
#endif
