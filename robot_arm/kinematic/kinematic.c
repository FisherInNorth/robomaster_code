/**
  ******************************************************************************
  * @file    Project/APP/kinematic.c 
  * @author  Siyuan Qiao&Junyu Luo 
  * @version V1.0.0
  * @date    2.2020
  * @brief   ���������˶�ѧ����
  *          ���ٶȵ�λ�� cm/s
  *          ���ٶȵ�λ�� rad/s
	*          ת�ٵ�λ��   rpm
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
	* @function����:BaseVel_To_WheelVel()
	* @brief����:���˶�ѧ��ʽ,����Ҫ�õ��ĵ����ٶ�ת��Ϊ���ӵ����ٶ�
	* @param����:���̵������ٶ�
	* @retval����ֵ:��
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
	* @function����:Get_Base_Velocities()
	* @brief����:���˶�ѧ��ʽ,ͨ����̥��ʵ��ת�ټ�����̼������ĵ������ٶ�
	* @param����:��
	* @retval����ֵ:��
  */
void Get_Base_Velocities(void)
{
	//���ݵ��ת�ٲ�������ת��
	Kinematics.wheel1.actual_speed.rpm = - motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =   motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = - motor4.actual_speed / M3508_REDUCTION_RATIO;
	//����ת��ת��Ϊ�������ٶ�
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
	//�������ٶ�ת��Ϊ��������������ٶ�
	Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*(half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
}



	/**
	* @function����:chassic_speed_control()
	* @brief����:�����ٶȿ��ƣ�������ٶ�ָ����㣬��ֵ��������
	* @param����:�����ٶ�
	* @retval����ֵ:��
  */
void chassic_speed_control(float speed_x, float speed_y, float speed_r)
{
		int max;
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);    //�ٶȻ���
 
		max=find_max(motor1.target_speed,motor2.target_speed,motor3.target_speed,motor4.target_speed);
		if(max>MAX_MOTOR_SPEED)
		{
			motor1.target_speed=(int)(motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
		}
		set_chassis_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);  //�ı��ٶ�pidĿ���ٶ�
}	

	/**
	* @function����:flip_speed_control()
	* @brief����:��תװ�õıջ�����
	* @param����:��ת������ٶ�
	* @retval����ֵ:��
  */
void flip_speed_control(float motor6_speed,float motor7_speed)    
{	
	set_flip_speed(motor6_speed,motor7_speed);
}


	/**
	* @function����:card_angle_control()
	* @brief����:��Ԯ���ıջ�����
	* @param����:��Ԯ�����ת�ĽǶ�
	* @retval����ֵ:��
  */
void card_angle_control(float card_angle)    
{
	card_angle=card_angle/360.0f*8191*36;
  	set_card_angle(resucecard_angle);
}
