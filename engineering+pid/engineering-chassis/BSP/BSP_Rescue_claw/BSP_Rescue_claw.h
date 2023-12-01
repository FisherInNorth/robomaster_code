#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "pid.h"
#include "BSP_Motor.h"

extern MOTOR_t claw_L_motor,claw_R_motor;

#define Claw_Target_Speed 3000
#define Claw_Kp						2.3
#define Claw_Ki						0.54
#define Claw_Kd						0.25

void Claw_Init(void);
void Claw_Task(MOTOR_MOVE_t claw_move);

#endif
