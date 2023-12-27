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


#include "kinematic.h"
#include "motor.h"
#include "speed_pid.h"
#include "algorithm.h"
#include "flag.h"

Kinematics_t Kinematics;


/**
  * @brief  ���˶�ѧ��ʽ,����Ҫ�õ��ĵ����ٶ�ת��Ϊ���ӵ����ٶ�
  */
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.wheel1.target_speed.linear_vel = linear_x + linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel = linear_x - linear_y - angular_z*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel = linear_x - linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = linear_x +  linear_y - angular_z*(half_width+half_length);
	//���ٶ� cm/s  תת��  RPM 
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
  * @brief  ���˶�ѧ��ʽ,ͨ����̥��ʵ��ת�ټ�����̼������ĵ������ٶ�
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
  * @brief  �����ٶȿ��ƣ�������ٶ�ָ����㣬��ֵ��������
  */
void chassic_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
	if(stop_flag_1 == 0 && speed_x == 0 && speed_y == 0 && speed_r == 0)
	{
		stop_flag_1 = 1;			//ֹͣ   �˱�־Ϊ�˱����ν���
		stop_chassis_motor();			//ͣ����  ���Ƕȱջ�
	}
	else if(speed_x != 0 || speed_y != 0 || speed_r != 0)
	{
		stop_flag_1 = 0;
		//�ٶȻ���
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
 
		max=find_max(motor1.target_speed,motor2.target_speed,motor3.target_speed,motor4.target_speed);
		if(max>max_motor_speed)
		{
			motor1.target_speed=(int)(motor1.target_speed*max_motor_speed*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*max_motor_speed*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*max_motor_speed*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*max_motor_speed*1.0/max);
		}
			//�ı��ٶ�pidĿ���ٶ�
			set_chassis_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
	}
}	

/**
  * @brief  ץ���ٶȿ���
  */	
void handle_speed_control(float motor5_speed,float motor6_speed)    //
{
	
 if(stop_flag_2 == 0 && motor5_speed==0)
	{
		stop_flag_2 = 1;			//ֹͣ   �˱�־Ϊ�˱����ν���
		stop_trigger_motor();			//ͣ����  ���Ƕȱջ�
	}
 else if(motor5_speed!=0)
	{
		stop_flag_2 = 0;
		trigger_to_motor(motor5_speed);
	  set_handle_speed(motor5_speed,motor6_speed);
  }
}
int angle_judge_flag1=0;
int angle_judge_flag2=0;
void handle_angle_control(float yaw_angle,float pitch_angle)
{
	yaw_angle = yaw_angle/360*8191*36;
	if(angle_judge_flag1==0)
	{
		motor5.apid.trigger_first_total_angle_storage=motor5.total_angle;
		angle_judge_flag1++;
	}
//	if(angle_judge_flag==0&&abs(motor5.apid.err>5000000))
//	{
//		motor5.apid.trigger_first_total_angle_storage=motor5.total_angle;
//		angle_judge_flag++;
//	}
//	if(abs(motor5.apid.err<5000000))
//		angle_judge_flag=0;
	pitch_angle = pitch_angle/360*8191*36;
	if(angle_judge_flag2==0)
	{
		motor6.apid.trigger_first_total_angle_storage=motor6.total_angle;
		angle_judge_flag2++;
	}
	set_handle_angle(yaw_angle,pitch_angle);
}

//void handle_current_control()
//		{
//			 motor6.vpid.PID_OUT=
//		}
void chassic_test()
{
if(test_flag==0)
 {
	Kinematics.target_velocities.linear_x=45;
  test_flag=1;
 } 
 if(test_flag==1&&motor1.round_cnt>=35)
 {
	 Kinematics.target_velocities.linear_x=-45;
   test_flag=2; 
 }
  if(test_flag==2&&motor1.round_cnt<=10)
 {
	 Kinematics.target_velocities.linear_x=0;
	 test_flag=3;
 }
  if(test_flag==3)
	{
	Kinematics.target_velocities.angular_z=1;
	test_flag=4;
	motor4.round_cnt=0;
	}
	if(test_flag==4&&motor4.round_cnt>=35)
	{
	Kinematics.target_velocities.angular_z=-1;
  test_flag=5;
	motor3.round_cnt=0;
	}
	if(test_flag==5&&motor3.round_cnt<=-45)
	{
	Kinematics.target_velocities.angular_z=0;
  }

}


