#include "pid.h"
#include "micromath.h"


/*���������ֱ�Ϊ��Kp��Ki��Kd�����ַ���ֵ�������޷�ֵ������޷�ֵ*/
/*�ĸ����̵�����ٶȻ�pid����*/
float ChassisMotor1_PID_Parameters[6] = {12.0, 0, 0, 10000, 10000, 10000};
float ChassisMotor2_PID_Parameters[6] = {8.0, 0, 0, 10000, 10000, 10000};
float ChassisMotor3_PID_Parameters[6] = {12.0, 0, 0, 10000, 10000, 10000};
float ChassisMotor4_PID_Parameters[6] = {8.0, 0, 0, 10000, 10000, 10000};

/*�������̧��������ٶȻ��ͽǶȻ�pid����*/
float Outboard_MotorL_vPID_Parameters[6] = {28	,	0.05	, 0 , 10000, 10000 , 15000};
float Outboard_MotorL_aPID_Parameters[6] = {0.5	, 0	, 0 , 10000, 100	 , 1700};
float Outboard_MotorR_vPID_Parameters[6] = {28	,	0.05	, 0 , 10000, 10000 , 15000};
float Outboard_MotorR_aPID_Parameters[6] = {0.5	, 0	, 0 , 10000, 100	 , 1700};

/*�ĸ���ʯ�˶�������ٶȻ�pid����*/
float MineralMotor_PID_Parameters[6] = {2 , 0.025, 5 , 10000, 10000, 6000};


void ChassisMotorPID_Calc(MOTOR_t *motor, float * PID_Parameters)
{
	Vpid_Calc(motor, PID_Parameters[0], PID_Parameters[1], PID_Parameters[2], PID_Parameters[3], PID_Parameters[4], PID_Parameters[5]);
}

void OutboardLiftMotor_PID_Calc(MOTOR_t *motor, float * aPID_Parameters, float * vPID_Parameters)
{
	Apid_Calc(motor, aPID_Parameters[0], aPID_Parameters[1], aPID_Parameters[2], aPID_Parameters[3], aPID_Parameters[4], aPID_Parameters[5]);
	
	motor->vpid.target_speed = motor->apid.PID_OUT;
	
	Vpid_Calc(motor, vPID_Parameters[0], vPID_Parameters[1], vPID_Parameters[2], vPID_Parameters[3], vPID_Parameters[4], vPID_Parameters[5]);
}

void MineralMotorPID_Calc(MOTOR_t *motor, float * PID_Parameters)
{
	Vpid_Calc(motor, PID_Parameters[0], PID_Parameters[1], PID_Parameters[2], PID_Parameters[3], PID_Parameters[4], PID_Parameters[5]);
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
