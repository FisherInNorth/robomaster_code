#include "pid.h"
#include "micromath.h"


/*���������ֱ�Ϊ��Kp��Ki��Kd�����ַ���ֵ�������޷�ֵ������޷�ֵ*/
/*�������������ƶ��ĵ�����ٶȻ��ͽǶȻ�pid����*/
float Longitudinal_MotorL_vPID_Parameters[6] = {0.5	,	0.5	, 10 , 10000, 10000 , 9000};
float Longitudinal_MotorL_aPID_Parameters[6] = {1	, 0	, 100 , 10000, 100	 , 1000};

float Longitudinal_MotorR_vPID_Parameters[6] = {0.5	,	0.5	, 10 , 10000, 10000 , 9000};
float Longitudinal_MotorR_aPID_Parameters[6] = {1	, 0	, 100 , 10000, 100	 , 1000};

/*�����ڲ�̧��������ٶȻ��ͽǶȻ�pid����*/
float Inside_MotorL_vPID_Parameters[6] = {0.5	,	0.5	, 0 , 10000, 10000 , 10000};
float Inside_MotorL_aPID_Parameters[6] = {1	, 0	, 100 , 10000, 100	 , 1000};

float Inside_MotorR_vPID_Parameters[6] = {0.5	,	0.5	, 0 , 10000, 10000 , 10000};
float Inside_MotorR_aPID_Parameters[6] = {1	, 0	, 100 , 10000, 100	 , 1000};

/*���ƺ����ƶ��ĵ�����ٶȻ��ͽǶȻ�pid����*/
float horizontal_motor_vPID_Parameters[6]={2	,	1.5	, 0 , 10000, 100 , 10000};
float horizontal_motor_aPID_Parameters[6]={2	,	0.8	, 0 , 10000, 100 , 1000};
/*����yaw���ٶȻ��ͽǶȻ�pid����*/
float chuck_yaw_vPID_Parameters[6] = {0.8	,	0.12	, 0 , 10000, 10000 , 3000};
float chuck_yaw_aPID_Parameters[6] = {0.5	, 0	, 70 , 10000, 100	 , 4000};
/*����pitch���ٶȻ��ͽǶȻ�pid����*/
float chuck_pitch_vPID_Parameters[6] = {0.8	,	0.12	, 0 , 10000, 10000 , 3000};
float chuck_pitch_aPID_Parameters[6] = {0.5	, 0	, 70 , 10000, 100	 , 4000};
/*����roll���ٶȻ��ͽǶȻ�pid����*/
float chuck_roll_vPID_Parameters[6] = {0.5	,	0	, 0 , 10000, 10000 , 3000};
float chuck_roll_aPID_Parameters[6] = {0.5	, 0	, 0 , 10000, 100	 , 4000}; 

extern float Ren;

void Longitudinal_Motor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
	
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void InsideLiftMotor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
	
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void ChuckYaw_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
	
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void ChuckPitch_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
	
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void ChuckRoll_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
	
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void HorizontalMotor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
//	motor->vpid.target_speed=Ren;
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void Vpid_Calc(MOTOR_t *motor, float kp, float ki, float kd, float IntegralSeparation, float Integral_max, float vPID_max)
{
	/*ʵ���ٶ���Ŀ���ٶȵĲ��*/
	motor->vpid.err = motor->vpid.target_speed - motor->vpid.actual_speed;
	
	/*���ַ��룬�������󲻲������*/
	if(abs(motor->vpid.err) <= IntegralSeparation)
	{
		motor->vpid.err_integration += motor->vpid.err;
	}
	
	/*�����޷�*/
	if(motor->vpid.err_integration > Integral_max)
	{
		motor->vpid.err_integration = Integral_max;
	}
	else if(motor->vpid.err_integration < -Integral_max)
	{
		motor->vpid.err_integration = -Integral_max;
	}
	
	motor->vpid.P_OUT = kp * motor->vpid.err;								//P��
	motor->vpid.I_OUT = ki * motor->vpid.err_integration;		//I��
	motor->vpid.D_OUT = kd * (motor->vpid.err-motor->vpid.last_err);//D��
	motor->vpid.last_err=motor->vpid.err;
	
	/*����޷�*/
	if(abs(motor->vpid.err) <= 2)
	{
	  motor->vpid.PID_OUT=0;
	}
	else
	{	
		if((motor->vpid.P_OUT+motor->vpid.I_OUT+motor->vpid.D_OUT) > vPID_max)
		{
			motor->vpid.PID_OUT = vPID_max;
		}
	  else if((motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT) < -vPID_max)
		{
			motor->vpid.PID_OUT = -vPID_max;
		}
	  else
		{
			motor->vpid.PID_OUT = motor->vpid.P_OUT + motor->vpid.I_OUT + motor->vpid.D_OUT;
		}
  }
}


void Apid_Calc(MOTOR_t *motor, float kp, float ki, float kd, float IntegralSeparation, float Integral_max, float aPID_max)
{
	/*ʵ�ʽǶ���Ŀ��ǶȵĲ��*/
	motor->apid.err = motor->apid.target_angle - motor->apid.total_angle;
	
	/*���ַ��룬�������󲻲������*/
	if(abs(motor->apid.err) <= IntegralSeparation)
	{
		motor->apid.err_integration += motor->apid.err;
	}
	
	/*�����޷�*/
	if(motor->apid.err_integration > Integral_max)
	{
		motor->apid.err_integration = Integral_max;
	}
	else if(motor->apid.err_integration < -Integral_max)
	{
		motor->apid.err_integration = -Integral_max;
	}
	
	motor->apid.P_OUT = kp * motor->apid.err;								//P��
	motor->apid.I_OUT = ki * motor->apid.err_integration;		//I��
	motor->apid.D_OUT = kd * (motor->apid.err-motor->apid.last_err);//D��
	motor->apid.last_err=motor->apid.err;
	
	/*����޷�*/
	if(abs(motor->apid.err) <= 2)
	{
	  motor->apid.PID_OUT=0;
	}
	else
	{	
		if((motor->apid.P_OUT+motor->apid.I_OUT+motor->apid.D_OUT) > aPID_max)
		{
			motor->apid.PID_OUT = aPID_max;
		}
	  else if((motor->apid.P_OUT + motor->apid.I_OUT + motor->apid.D_OUT) < -aPID_max)
		{
			motor->apid.PID_OUT = -aPID_max;
		}
	  else
		{
			motor->apid.PID_OUT = motor->apid.P_OUT + motor->apid.I_OUT + motor->apid.D_OUT;
		}
  }
}
