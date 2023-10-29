#ifndef __HANDLE_H__
#define __HANDLE_H__
#include "pid.h"
#include "BSP_Motor.h"

extern MOTOR_t handle_motor;

void Handle_Init(void);
void Handle_Task(MOTOR_MOVE_t handle_move);

#endif
