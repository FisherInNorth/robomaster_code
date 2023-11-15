#ifndef __PUSH_H__
#define __PUSH_H__

#include "BSP_Motor.h"
#include "tim.h"
void Push_Task(MOTOR_MOVE_t push_moveF, MOTOR_MOVE_t push_moveB);
void TIM_Push_PWM_Init(void);

#endif
