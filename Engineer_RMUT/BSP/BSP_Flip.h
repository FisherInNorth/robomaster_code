#ifndef __FLIP_H__
#define __FLIP_H__
#include "pid.h"
#include "BSP_Motor.h"
#include "motor.h"

extern MOTOR_t flip_R_motor;
extern MOTOR_t flip_L_motor;
extern MOTOR_MOVE_t flip_move;

void Flip_Init(void);
void Flip_Task(MOTOR_MOVE_t flip_move);
void Set_Flip_Angle_L(int target_angle);
void Set_Flip_Angle_R(int target_angle);


#endif
