/**
  ******************************************************************************
  * @file    Project/HARDWARE/motor.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................

*/  
#include "motor.h"
#include "can.h"
#include "delay.h"
#include "angle_pid.h"
#include "kinematic.h"

MOTOR_t motor1,motor2,motor3,motor4,motor5,motor6,stepper_motor_left,stepper_motor_right,limit_switch;
LOOPBACK loopback;

int max_motor_speed=MAX_MOTOR_SPEED;		//���������ٶ�
float max_base_linear_speed=MAX_BASE_LINEAR_SPEED;  //��������������ٶ� 
float max_base_rotational_speed=MAX_BASE_ROTATIONAL_SPEED;  //�������������ٶ�
int callback_flag=1;

/**
  * @breif ���can�ص���ֵ����
	*/
void record_motor_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = speed;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}
	
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;// - motor->start_angle;
}
/**********************��ֵ�˲�****************************/
/*  @function name:record_gimbal_callback()
    @author:junyu luo
    @date:2021.1.5
    @instruction:��������can���ߵ����ݣ�ȥ�������е�������Сֵ��ʣ�µ���Ϊ�������ݣ����������ݴ�������Ŀ������仯
*/     
void record_gimbal_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	static int temp_speed1,temp_speed2,temp_speed3,temp_angle1,temp_angle2,temp_angle3;
	static int maxangle,minangle,maxspeed,minspeed;
	motor->last_angle = motor->actual_angle;
	motor->actual_current = current;
	//motor->actual_speed = speed;
	//motor->actual_current = current;

	switch(callback_flag)
	{
		case(1):
		{
		temp_angle1=angle;
		temp_speed1=speed;
		callback_flag=2;
	  }
	break;
		case(2):
	{
		temp_angle2=angle;
		temp_speed2=speed;
		callback_flag=3;
	}
	break;
		case(3):
	{
		temp_angle3=angle;
		temp_speed3=speed;
		maxspeed=(temp_speed1>temp_speed2?temp_speed1:temp_speed2);
		maxspeed=(maxspeed>temp_speed3?maxspeed:temp_speed3);
		minspeed=(temp_speed1<temp_speed2?temp_speed1:temp_speed2);
		minspeed=(maxspeed<temp_speed3?minspeed:temp_speed3);

		maxangle=(temp_angle1>temp_angle2?temp_angle1:temp_angle2);
		maxangle=(maxangle>temp_angle3?maxspeed:temp_angle3);
		minangle=(temp_angle1<temp_angle2?temp_angle1:temp_angle2);
		minangle=(maxangle<temp_angle3?minangle:temp_angle3);

		motor->actual_angle =(temp_angle1+temp_angle2+temp_angle3)-maxangle-minangle;
	  motor->actual_speed =(temp_speed1+temp_speed2+temp_speed3)-maxspeed-minspeed;
		
		callback_flag=1;
	}
	break;
	default:break;
}
}
	

/**
  * @breif ���������ʼ��
  */
void motor_init()
{
	//1�ŵ��.
	motor1.start_angle = 0;
	motor1.actual_angle = 0;
	motor1.actual_speed = 0;
	motor1.start_angle_flag = 0;
	motor1.actual_current = 0;
	motor1.target_current = 0;
	//motor1.temp = 0;

	//2�ŵ��
	motor2.start_angle = 0;
	motor2.actual_angle = 0;
	motor2.start_angle_flag = 0;
	motor2.actual_speed = 0;
	motor2.actual_current = 0;
	motor2.target_current = 0;
	//motor2.temp = 0;

	//3�ŵ��
	motor3.start_angle = 0;
	motor3.actual_angle = 0;
	motor3.start_angle_flag = 0;
	motor3.actual_speed = 0;
	motor3.actual_current = 0;
	motor3.target_current = 0;
	//motor3.temp = 0;

	//4�ŵ��
  motor4.start_angle = 0;
	motor4.actual_angle = 0;
	motor4.start_angle_flag = 0;
	motor4.actual_speed = 0;
	motor4.actual_current = 0;
	motor4.target_current = 0;
	//motor4.temp = 0;
	
	Kinematics.stepper_motor_left.pulse_num=0;
	Kinematics.stepper_motor_left.pwm_pulse=0;
	Kinematics.stepper_motor_right.pulse_num=0;
	Kinematics.stepper_motor_right.pwm_pulse=0;	
 
}


/**
  * @breif ���������ֵ����
  */
void set_chassis_current()
{
	u8 current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	motor1.target_current = motor1.vpid.PID_OUT;
	motor2.target_current = motor2.vpid.PID_OUT;
	motor3.target_current = motor3.vpid.PID_OUT;
	motor4.target_current = motor4.vpid.PID_OUT;

	
	//can����ͨ��Э�飬���յ��˵����
	current_msg[0] = motor1.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = motor1.target_current & 0xff;		//1�ŵ��������8λ
	current_msg[2] = motor2.target_current >> 8;			//2�ŵ��������8λ
	current_msg[3] = motor2.target_current & 0xff;		//2�ŵ��������8λ
	current_msg[4] = motor3.target_current >> 8;			//3�ŵ��������8λ
	current_msg[5] = motor3.target_current & 0xff;		//3�ŵ��������8λ
	current_msg[6] = motor4.target_current >> 8;			//4�ŵ��������8λ
	current_msg[7] = motor4.target_current & 0xff;		//4�ŵ��������8λ
	
	//can��������֡
	CAN1_Send_CHASSIS_Msg(current_msg);
}

void set_handle_current()
{
	u8 current_msg[8];
	
	//���Ŀ�����Ϊ�ٶ�pid���
	motor5.target_current = motor5.vpid.PID_OUT;//
	motor6.target_current = motor6.vpid.PID_OUT;//
	
	//can����ͨ��Э�飬���յ��˵����
	current_msg[0] = motor5.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = motor5.target_current & 0xff;		//1�ŵ��������8λ
	current_msg[2] = motor6.target_current >> 8;			//1�ŵ��������8λ
	current_msg[3] = motor6.target_current & 0xff;		//1�ŵ��������8λ

	//can��������֡
	CAN1_Send_handle_Msg(current_msg);
}



/**
  * @breif �����̵��ֹͣ����ǰ�Ƕ�
  */
void stop_chassis_motor()
{
	//��ȡ��ǰ�Ƕ�ֵ
	motor1.stop_angle = motor1.actual_angle;
	motor2.stop_angle = motor2.actual_angle;
	motor3.stop_angle = motor3.actual_angle;
	motor4.stop_angle = motor4.actual_angle;
	
	//�ı�Ƕ�pidĿ��Ƕ�ֵ
	set_chassis_motor_angle(motor1.stop_angle,motor2.stop_angle,motor3.stop_angle,motor4.stop_angle);
	
}

/**
  * @breif �������ֵ��ֹͣ����ǰ�Ƕ�
  */
void stop_trigger_motor()
{
motor5.stop_angle = motor5.actual_angle;
	
set_trigger_motor_angle(motor5.stop_angle);

}


void handle_90()
{
	Kinematics.handle_L.target_angle=-80;
	Kinematics.handle_R.target_angle=-80;
}

void handle_180()
{
	Kinematics.handle_L.target_angle=-200;
	Kinematics.handle_R.target_angle=-200;
}


