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
#include "pid.h"
#include "BSP_motor.h"
#include "algorithm.h"
#include "stm32F4xx.h"
#include "stdint.h"



/**
* @function����:vpid_realize()
* @brief����:vpid�ļ��㹫ʽ
* @param����:pid����
* @retval����ֵ:��
 */

void Vpid_Realize(MOTOR_t motor,float kp,float ki,float kd)
{
	motor.vpid.err = motor.vpid.target_speed - motor.vpid.actual_speed;
	if(abs(motor.vpid.err) <= motor.integralSeparation)		//���ַ���
		motor.vpid.err_integration += motor.vpid.err;
	if(motor.vpid.err_integration > motor.integral_max)		//�����ֱ���
		motor.vpid.err_integration = motor.integral_max;
	else if(motor.vpid.err_integration < -motor.integral_max)
		motor.vpid.err_integration = -motor.integral_max;
	
	motor.vpid.P_OUT = kp * motor.vpid.err;								//P��
	motor.vpid.I_OUT = ki * motor.vpid.err_integration;		//I��
	motor.vpid.D_OUT = kd * (motor.vpid.err-motor.vpid.last_err);//D��
	motor.vpid.last_err=motor.vpid.err;
	//����޷�
	if(abs(motor.vpid.err) <= 2)
	  motor.vpid.PID_OUT=0;
	else
	{	
		if((motor.vpid.P_OUT + motor.vpid.I_OUT + motor.vpid.D_OUT)> motor.vPID_max) 
			motor.vpid.PID_OUT = motor.vPID_max;
	  	else if((motor.vpid.P_OUT + motor.vpid.I_OUT + motor.vpid.D_OUT) < -motor.vPID_max) 
			motor.vpid.PID_OUT = -motor.vPID_max;
	  	else
			motor.vpid.PID_OUT = motor.vpid.P_OUT + motor.vpid.I_OUT + motor.vpid.D_OUT;
    }
}

	/**
	* @function����:apid_realize()
	* @brief����:apid�ļ��㹫ʽ
	* @param����:pid����
	* @retval����ֵ:��
  */
void Apid_Realize(MOTOR_t motor,float kp,float ki,float kd)
{

	motor.apid.err = motor.apid.target_angle - motor.apid.total_angle;
	motor.apid.P_OUT = kp * motor.apid.err;
	motor.apid.D_OUT = kd * (motor.apid.err-motor.apid.last_err);
	motor.apid.last_err = motor.apid.err;
  if((motor.apid.P_OUT + motor.apid.D_OUT) > motor.aPID_OUT_MAX) 
  {
			  
	  motor.apid.PID_OUT = motor.aPID_OUT_MAX;
	}
	else if((motor.apid.P_OUT + motor.apid.D_OUT) < -motor.aPID_OUT_MAX) 
	{
		motor.apid.PID_OUT = -motor.aPID_OUT_MAX;
	}
	else
	{
		motor.apid.PID_OUT = motor.apid.P_OUT + motor.apid.D_OUT;
	}
}


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
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}
