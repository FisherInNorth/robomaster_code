#include "kinematics.h"

Kinematics_t Kinematics;

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.wheel1.target_speed.linear_vel = -linear_x - linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel = -linear_x + linear_y - angular_z*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel = -linear_x + linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = -linear_x - linear_y - angular_z*(half_width+half_length);
	//���ٶ� cm/s  תת��  RPM 
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel4.target_speed.rpm = Kinematics.wheel4.target_speed.linear_vel * VEL2RPM;
	
	chassis_motor1.vpid.target_speed = - (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	chassis_motor2.vpid.target_speed =   (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
	chassis_motor3.vpid.target_speed = - (int)(Kinematics.wheel3.target_speed.rpm * M3508_REDUCTION_RATIO);
	chassis_motor4.vpid.target_speed =   (int)(Kinematics.wheel4.target_speed.rpm * M3508_REDUCTION_RATIO);
	
}
