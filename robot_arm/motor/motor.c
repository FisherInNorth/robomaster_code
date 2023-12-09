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
	// * ��Χ  ��3508  ���ݷ�Χ                       ����ͷת��Ϊ0.22rps,��13.3rpm
//						�������� -16384 ~ 0 ~ +16384
//						ת�ӽǶ�	0~8191 �ȼ�	0-360��
//						ת��ת��	max=450	RPM
//						ʵ��ת�ص���
//						����¶�
//			   ��GM6020���ݷ�Χ
//						������ѹ -30000 ~ 0 ~ +30000
//						ת�ӽǶ�	0~8191 �ȼ�	0-360��
//						ת��ת��	max=350	RPM
//						ʵ��ת�ص���
//						����¶�
//			   ��P2006 ���ݷ�Χ
//						�������� -10000 ~ 0 ~ +10000
//						ת�ӽǶ�	0~8191 �ȼ�	0-360��
//						ת��ת��	max=450	RPM
//						ʵ�����ת��
      ..................NEUQ_SUDO..................

*/
#include "motor.h"
#include "pid.h" 

/**
* @brief  ���can�ص���ֵ����
* @param MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current
* @retval void
* @attention
*/
void record_motor_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->apid.actual_angle;
	motor->apid.actual_angle = angle;
	motor->vpid.actual_speed = speed;
	motor->actual_current = current;
	if(motor->start_angle_flag == 0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}

	if(motor->apid.actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->apid.actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}

/**
* @brief  ���е��������ʼ��
* @param 
* @retval 
* @attention
*/
void motor_Init()
{
	motor_all_init(&motor1);
	motor_all_init(&motor2);
	motor_all_init(&motor3);
	motor_all_init(&motor4);
	motor_all_init(&motor5);
	motor_all_init(&motor6);
	motor_all_init(&motor7);
}


/**
* @brief  ���е��������ʼ��
* @param 
* @retval 
* @attention
*/
void motor_all_init(MOTOR_t *motor)
{
	//�����ʼ�� 
	motor->start_angle = 0;
	motor->apid.actual_angle = 0;
	motor->vpid.actual_speed = 0;
	motor->start_angle_flag = 0;
	motor->actual_current = 0;
	motor->target_current = 0;
}

/**
* @brief  chassis������ֵ������can1��
* @param void
* @retval void
* @attention
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

/**
* @brief  handle������ֵ������can1��
* @param void
* @retval void
* @attention
*/
void set_rescuecard_current()
{
	u8 current_msg[8];

	//���Ŀ�����Ϊ�ٶ�pid���
	motor5.target_current = motor5.vpid.PID_OUT;

	//can����ͨ��Э�飬���յ��˵����
	current_msg[0] = motor5.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = motor5.target_current & 0xff;		//1�ŵ��������8λ

	//can��������֡
	CAN1_Send_rescuecard_Msg(current_msg);
}

/**
* @brief  flip������ֵ������can1��
* @param void
* @retval void
* @attention
*/
void set_rescueclaw_current()
{
	u8 current_msg[8];

	//���Ŀ�����Ϊ�ٶ�pid���
	motor6.target_current = motor6.vpid.PID_OUT;
	motor7.target_current = motor7.vpid.PID_OUT;

	//can����ͨ��Э�飬���յ��˵����
	current_msg[2] = motor6.target_current >> 8;			//1�ŵ��������8λ
	current_msg[3] = motor6.target_current & 0xff;		//1�ŵ��������8λ
	current_msg[4] = motor7.target_current >> 8;			//1�ŵ��������8λ
	current_msg[5] = motor7.target_current & 0xff;		//1�ŵ��������8λ

	//ʹ��can1��������֡
	CAN1_Send_rescueclaw_Msg(current_msg);
}

