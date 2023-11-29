#ifndef __FLIP_H__
#define __FLIP_H__
#include "pid.h"
#include "BSP_Motor.h"

#define Card_Target_Speed 		2000
#define Card_Kp						    8
#define Card_Ki					    	1.2
#define Card_Kd						    0.5

void Card_Init(void);
void Card_Task(MOTOR_MOVE_t card_move);
extern MOTOR_t card_motor;
void CAN1_Card_Msg(uint8_t *mdata);
#endif
