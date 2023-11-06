#ifndef __HANDLE_H__
#define __HANDLE_H__
#include "pid.h"
#include "BSP_Motor.h"

extern MOTOR_t handle_motor;

#define Handle_Target_Speed 3000
#define Handle_Kp						4
#define Handle_Ki						0.8
#define Handle_Kd						1.6

void Handle_Init(void);
void Handle_Task(MOTOR_MOVE_t handle_move);

#endif
