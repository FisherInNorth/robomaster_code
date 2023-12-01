#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "pid.h"
#include "kinematics.h"
#include "BSP_Motor.h"

#define set_chassis_speed(motor1_speed,motor2_speed,motor3_speed,motor4_speed) \
        do{                                                                    \
					chassis_motor1.vpid.target_speed = motor1_speed;	     \
	        chassis_motor2.vpid.target_speed = motor2_speed;      \
	        chassis_motor3.vpid.target_speed = motor3_speed;      \
	        chassis_motor4.vpid.target_speed = motor4_speed;      \
				}while(0)                                                              \



extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
void chassis_move(void);

#endif



