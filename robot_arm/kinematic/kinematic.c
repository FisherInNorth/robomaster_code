/**
  ******************************************************************************
  * @file    Project/APP/kinematic.c 
  * @author  Siyuan Qiao&Junyu Luo 
  * @version V1.0.0
  * @date    2.2020
  * @brief   底盘正逆运动学演算
  *          线速度单位： cm/s
  *          角速度单位： rad/s
	*          转速单位：   rpm
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "stm32f4xx_gpio.h"
#include "remote_code.h"
#include "kinematic.h"
#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "algorithm.h"
#include "flag.h"
#include "remote_code.h"
#include "delay.h"
#include "push.h"
#include "key.h"

Kinematics_t Kinematics;
	
	/**
	* @function函数:BaseVel_To_WheelVel()
	* @brief描述:逆运动学公式,把想要得到的底盘速度转换为轮子的线速度
	* @param输入:底盘的三轴速度
	* @retval返回值:无
  */
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.wheel1.target_speed.linear_vel = linear_x + linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel = linear_x - linear_y - angular_z*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel = linear_x - linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = linear_x +  linear_y - angular_z*(half_width+half_length); 
	
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel4.target_speed.rpm = Kinematics.wheel4.target_speed.linear_vel * VEL2RPM;
	
	motor1.target_speed =   (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor2.target_speed = - (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor3.target_speed =   (int)(Kinematics.wheel3.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor4.target_speed = - (int)(Kinematics.wheel4.target_speed.rpm * M3508_REDUCTION_RATIO);
}



	/**
	* @function函数:Get_Base_Velocities()
	* @brief描述:正运动学公式,通过轮胎的实际转速计算底盘几何中心的三轴速度
	* @param输入:无
	* @retval返回值:无
  */
void Get_Base_Velocities(void)
{
	//根据电机转速测算轮子转速
	Kinematics.wheel1.actual_speed.rpm = - motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =   motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = - motor4.actual_speed / M3508_REDUCTION_RATIO;
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



	/**
	* @function函数:chassic_speed_control()
	* @brief描述:底盘速度控制，将宏观速度指令解算，赋值到具体电机
	* @param输入:底盘速度
	* @retval返回值:无
  */
void chassic_speed_control(float speed_x, float speed_y, float speed_r)
{
		int max;
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);    //速度换算
 
		max=find_max(motor1.target_speed,motor2.target_speed,motor3.target_speed,motor4.target_speed);
		if(max>MAX_MOTOR_SPEED)
		{
			motor1.target_speed=(int)(motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
		}
		set_chassis_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);  //改变速度pid目标速度
}	

	/**
	* @function函数:flip_speed_control()
	* @brief描述:翻转装置的闭环控制
	* @param输入:翻转电机的速度
	* @retval返回值:无
  */
void flip_speed_control(float motor6_speed,float motor7_speed)    
{	
	set_flip_speed(motor6_speed,motor7_speed);
}


	/**
	* @function函数:card_angle_control()
	* @brief描述:救援卡的闭环控制
	* @param输入:救援卡电机转的角度
	* @retval返回值:无
  */
void card_angle_control(float card_angle)    
{
	card_angle=card_angle/360.0f*8191*36;
  	set_card_angle(resucecard_angle);
}
