#include "ChassisMove_Task.h"

int16_t chassis_vx;
int16_t chassis_vy;
int16_t chassis_wz;

void ChassisMove(void)
{
	//int max;
		//速度换算，运动学分解
	BaseVel_To_WheelVel(chassis_vx, chassis_vy, chassis_wz);
	
	//max=find_max();
//	if(max>MAX_MOTOR_SPEED)
//	{
//		chassis_motor1.target_speed=(int)(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
//		chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
//		chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
//		chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
//	}
//	set_chassis_speed(chassis_motor1.target_speed, chassis_motor2.target_speed, chassis_motor3.target_speed, chassis_motor4.target_speed);
}	
