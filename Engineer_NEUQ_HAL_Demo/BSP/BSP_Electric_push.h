#ifndef __PUSH_H__
#define __PUSH_H__

#include "BSP_Motor.h"
#include "tim.h"
void Push_Task(MOTOR_MOVE_t push_move1, MOTOR_MOVE_t push_move2);
void TIM_Push_PWM_Init(void);

#endif
