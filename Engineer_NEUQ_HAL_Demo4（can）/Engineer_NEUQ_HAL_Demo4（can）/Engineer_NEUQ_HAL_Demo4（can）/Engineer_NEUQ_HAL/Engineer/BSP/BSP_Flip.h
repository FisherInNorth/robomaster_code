#ifndef __FLIP_H__
#define __FLIP_H__
#include "pid.h"
#include "BSP_Motor.h"
#include "motor.h"

extern MOTOR_t flip_R_motor;
extern MOTOR_t flip_L_motor;
extern MOTOR_MOVE_t flip_move;


#define Flip_Out_Speed 		2000
#define Flip_In_Speed 		-1500
#define Flip_Kp						4
#define Flip_Ki						0.8
#define Flip_Kd						1.6

void Flip_Init(void);
void Flip_Task(MOTOR_MOVE_t flip_move);

#endif
