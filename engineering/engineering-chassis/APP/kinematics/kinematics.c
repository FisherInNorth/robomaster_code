#include "kinematics.h"
Kinematics_t Kinematics;

extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4;
void Get_Base_Velocities(void);

//正运动学公式
//通过轮胎的实际转速计算底盘几何中心的三轴速度
void Get_Base_Velocities(void)
{
	//根据电机转速测算轮子转速
	Kinematics.wheel1.actual_speed.rpm = - chassis_motor1.vpid.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   chassis_motor2.vpid.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =   chassis_motor3.vpid.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = - chassis_motor4.vpid.actual_speed / M3508_REDUCTION_RATIO;
	//轮子转速转换为轮子线速度
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
	//轮子线速度转换为底盘中心三轴的速度
	Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*(half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
}


