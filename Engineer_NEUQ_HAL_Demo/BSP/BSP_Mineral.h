#ifndef __MINERAL_H__
#define __MINERAL_H__
#include "pid.h"
#include "BSP_Motor.h"

extern MOTOR_MOVE_t mineral_longitudinal_move;
extern MOTOR_MOVE_t mineral_widthwise_move;


//#define mineral_F_motor.vpid.kp 	4
//#define mineral_F_motor.vpid.ki 	0.8
//#define mineral_F_motor.vpid.kd 	1.6
//#define mineral_F_motor.vpid.target_speed		30*36

//#define mineral_pid_init()\
// do{\
//	 	mineral_F_motor.vpid.kp =4;\
//		 mineral_F_motor.vpid.ki=0.8;\
//	 	mineral_F_motor.vpid.kd=1.6;\
//		 mineral_F_motor.vpid.target_speed=Mineral_Longitudinal_Speed*36;\
//		}while(0)\

void Mineral_Init(void);
void Mineral_Task_Longitudinal(MOTOR_MOVE_t mineral_longitudinal_move);
void Mineral_Task_Widthwise(MOTOR_MOVE_t mineral_widthwise_move);
#endif
