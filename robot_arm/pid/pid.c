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


// ����: VPID_Init()
// ����: ����ٶ�pid������ʼ��
// ����������ٶ�pid�����ṹ��
// �������
// �ڲ��������û��������
void VPID_Init(VPID_t *vpid)
{
	vpid->target_speed=0;
	vpid->actual_speed=0;
	vpid->err=0;
	vpid->last_err=0;
	vpid->err_integration=0;
	vpid->P_OUT=0;
	vpid->I_OUT=0;
	vpid->D_OUT=0;
	vpid->PID_OUT=0;
}

// ����: VPID_Init_All()
// ����: ���е���ٶ�pid������ʼ��
// ��������
// �������
void VPID_Init_All()	
{
	VPID_Init(&motor1.vpid);
	VPID_Init(&motor2.vpid);
	VPID_Init(&motor3.vpid);
	VPID_Init(&motor4.vpid);
	VPID_Init(&motor5.vpid);
	VPID_Init(&motor6.vpid);
	VPID_Init(&motor7.vpid);
}




// ����: APID_Init()
// ����: �����е�Ƕ�pid������ʼ��
// �����������е�Ƕ�pid�����ṹ��
// �������
// �ڲ��������û��������
void APID_Init(APID_t *apid)
{
	apid->actual_angle=0;
	apid->target_angle=0;
	apid->total_angle=0;
	apid->err=0;
	apid->err_integration=0;
	apid->P_OUT=0;
	apid->I_OUT=0;
	apid->D_OUT=0;
	apid->PID_OUT=0;
}

// ����: APID_Init_All()
// ����: ���е����е�Ƕ�pid������ʼ��
// ��������
// �������
void APID_Init_All()
{
	APID_Init(&motor1.apid);
	APID_Init(&motor2.apid);
	APID_Init(&motor3.apid);
	APID_Init(&motor4.apid);
  APID_Init(&motor5.apid);
	APID_Init(&motor6.apid);
	APID_Init(&motor7.apid);
}

/**
* @function����:vpid_realize()
* @brief����:vpid�ļ��㹫ʽ
* @param����:pid����
* @retval����ֵ:��
 */

void vpid_realize(MOTOR_t *motor,float kp,float ki,float kd)
{
	motor->vpid.err = motor->vpid.target_speed - motor->vpid.actual_speed;
	if(abs(motor->vpid.err) <= motor->integralSeparation)		//���ַ���
		motor->vpid.err_integration += motor->vpid.err;
	if(motor->vpid.err_integration > motor->integral_max)		//�����ֱ���
		motor->vpid.err_integration = motor->integral_max;
	else if(motor->vpid.err_integration < -motor->integral_max)
		motor->vpid.err_integration = -motor->integral_max;
	
	motor->vpid.P_OUT = kp * motor->vpid.err;								//P��
	motor->vpid.I_OUT = ki * motor->vpid.err_integration;		//I��
	motor->vpid.D_OUT = kd * (motor->vpid.err-motor->vpid.last_err);//D��
	motor->vpid.last_err=motor->vpid.err;
	//����޷�
	if(abs(motor->vpid.err) <= 2)
	  motor->vpid.PID_OUT=0;
	else
	{	
		if((motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT)> motor->vPID_max) 
			motor->vpid.PID_OUT = motor->vPID_max;
	  	else if((motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT) < -motor->vPID_max) 
			pid->PID_OUT = -motor->vPID_max;
	  	else
			motor->vpid.PID_OUT = motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT;
    }
}

	/**
	* @function����:apid_realize()
	* @brief����:apid�ļ��㹫ʽ
	* @param����:pid����
	* @retval����ֵ:��
  */
void apid_realize(MOTOR_t *motor,float kp,float ki,float kd)
{

	motor->apid.err = motor->apid.target_angle - motor->apid.total_angle;
	motor->apid.P_OUT = kp * motor->apid.err;
	motor->apid.D_OUT = kd * (motor->apid.err-motor->apid.last_err);
	motor->apid.last_err = motor->apid.err;
  if((motor->apid.P_OUT + motor->apid.D_OUT) > aPID_OUT_MAX) 
  {
			  
	  motor->apid.PID_OUT = aPID_OUT_MAX;
	}
	else if((motor->apid.P_OUT + motor->apid.D_OUT) < -aPID_OUT_MAX) 
	{
		motor->apid.PID_OUT = -aPID_OUT_MAX;
	}
	else
	{
		motor->apid.PID_OUT = motor->apid.P_OUT + motor->apid.D_OUT;
	}
	set_speed(motor, motor->apid.PID_OUT);
}

void set_angle(MOTOR_t *motor, motor_can2_1_angle) 
{
	motor->apid.target_angle = motor_can2_1_angle;
} 

void set_speed(MOTOR_t *motor, motor_speed) 
{
	motor->vpid.target_speed = motor_speed;
}


