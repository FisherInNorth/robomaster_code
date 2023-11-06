#ifndef __MINERAL_H__
#define __MINERAL_H__
#include "pid.h"
#include "BSP_Motor.h"

extern MOTOR_MOVE_t mineral_longitudinal_move;
extern MOTOR_MOVE_t mineral_widthwise_move;

#define Mineral_Longitudinal_Speed 		2000
#define Mineral_Widthwise_Speed 			2000
#define Mineral_Kp						4
#define Mineral_Ki						0.8
#define Mineral_Kd						1.6

void Mineral_Init(void);
void Mineral_Task(MOTOR_MOVE_t mineral_longitudinal_move,
									MOTOR_MOVE_t mineral_widthwise_move);

#endif
