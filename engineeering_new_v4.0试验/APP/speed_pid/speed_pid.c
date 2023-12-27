/**
  ******************************************************************************
  * @file    Project/APP/speed_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid �����ļ�
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "speed_pid.h"

PID_t pid_t;
//int vpid_out_max=vPID_OUT_MAX;
enum switch_flag_t switch_flag;

/**
  * @breif ���ת��pid������ʼ��
	* @attention �ڲ��������������
	*/
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

/**
  * @breif ����ٶȻ�pid��ʼ��
  */
void VPID_Init_All()	
{
	VPID_Init(&motor1.vpid);
	VPID_Init(&motor2.vpid);
	VPID_Init(&motor3.vpid);
	VPID_Init(&motor4.vpid);
	VPID_Init(&motor5.vpid);
	VPID_Init(&motor6.vpid);
}


/**
  * @breif �ٶȻ�pid���㹫ʽ��΢����һ�㲻��Ҫ
  * @attention �ڲ������û��������
  */
void vpid_realize(VPID_t *vpid,float kp,float ki,float kd)
{
		vpid->err = vpid->target_speed - vpid->actual_speed;
		
 switch(switch_flag)
 {
	 case(CHASSIC):
	 {
		if(abs(vpid->err) <= CHASSIC_IntegralSeparation)		//���ַ���
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > CHASSIC_Integral_max)		//�����ֱ���
		vpid->err_integration = CHASSIC_Integral_max;
	  else if(vpid->err_integration < -CHASSIC_Integral_max)
		vpid->err_integration = -CHASSIC_Integral_max;
		
		vpid->P_OUT = kp * vpid->err;								//P��
	  vpid->I_OUT = ki * vpid->err_integration;		//I��
		
		//����޷�
		if((vpid->P_OUT + vpid->I_OUT )> CHASSIC_vPID_max) 
		vpid->PID_OUT = CHASSIC_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIC_vPID_max) 
		vpid->PID_OUT = -CHASSIC_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
	 case(HANDLE):
	 {
	 	if(abs(vpid->err) <= GIMBAL_IntegralSeparation)		//���ַ���
		vpid->err_integration += vpid->err;
	  if(vpid->err_integration > GIMBAL_Integral_max)		//�����ֱ���
		vpid->err_integration = GIMBAL_Integral_max;
	  else if(vpid->err_integration < -GIMBAL_Integral_max)
		vpid->err_integration = -GIMBAL_Integral_max;
	
	  vpid->P_OUT = kp * vpid->err;								//P��
	  vpid->I_OUT = ki * vpid->err_integration;		//I��
	  vpid->D_OUT = kd * (vpid->err-vpid->last_err);//D��
	  vpid->last_err=vpid->err;
	  //����޷�
	  if(abs(vpid->err) <= 2)
	  vpid->PID_OUT=0;
	  else
	  {	
	  if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT)> GIMBAL_vPID_max) 
		vpid->PID_OUT = GIMBAL_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT + vpid->D_OUT) < -GIMBAL_vPID_max) 
		vpid->PID_OUT = -GIMBAL_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT + vpid->D_OUT;
    }
	 }
	  break;
		default:break;
  }		
}

/**
  * @breif ����pid�ٶȻ����㺯��
  */
void vpid_chassic_realize(float kp,float ki,float kd)
{
	//��ȡ�����ǰת��
	motor1.vpid.actual_speed = motor1.actual_speed;
	motor2.vpid.actual_speed = motor2.actual_speed;
	motor3.vpid.actual_speed = motor3.actual_speed;
	motor4.vpid.actual_speed = motor4.actual_speed;
	
	switch_flag = CHASSIC;
	//�������ֵ
	vpid_realize(&motor1.vpid,kp,ki,kd);
	vpid_realize(&motor2.vpid,kp,ki,kd);
	vpid_realize(&motor3.vpid,kp,ki,kd);
	vpid_realize(&motor4.vpid,kp,ki,kd);
	
	switch_flag = NUL;
//���ʿ��Ʒ��������ԣ�
/*	power_limitation_jugement();
	power_limitation_coefficient();*/
}
/**
  * @breif ץ��pid�ٶȻ����㺯��
  */
void vpid_handle_realize(float kp,float ki,float kd)
{
	//��ȡ�����ǰת��
	motor5.vpid.actual_speed = motor5.actual_speed;
	motor6.vpid.actual_speed = motor6.actual_speed;
	
	switch_flag = HANDLE;
	
	vpid_realize(&motor5.vpid,kp,ki,kd);  //��ʼpid����
	//vpid_realize(&motor6.vpid,kp,ki,kd);
	
	switch_flag = NUL;
}

/************����Ϊ�����ó���***************/
int pid_target_speed=0;//�ٶȻ����Ա���
int pid_target_angle=4096;  //λ�û����Ա���

