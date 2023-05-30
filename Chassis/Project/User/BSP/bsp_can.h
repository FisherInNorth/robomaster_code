#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "Motor.h"
#include "bsp_led.h"


extern unsigned char CAN_RxData[8];

void CAN_Filter_Config(void);
void CAN1_Send_Msg(uint32_t StdId, unsigned char * data);
void CAN2_Send_Msg(uint32_t StdId, unsigned char * data);

#endif
