#include "BSP_Flip.h"
#include "BSP_Motor.h"
#include "BSP_Can.h"

MOTOR_MOVE_t flip_move;
MOTOR_t flip_R_motor;
MOTOR_t flip_L_motor;

int Flip_Out_Speed=20;
int Flip_In_Speed=-15;
float Flip_Kp=4.5;
float Flip_Ki=1.2;
float Flip_Kd=1.6;
int FLIP_SPEED=0;//������
/**
	* @function����:Flip_Init
	* @brief����:��е�۳�ʼ��
	* @param����:��
	* @retval����ֵ:��
  */
void Flip_Init(void)
{
	flip_R_motor.target_current=0;
	flip_R_motor.actual_current=0;
	flip_R_motor.round_cnt=0.0f;
	flip_R_motor.vpid.target_speed=0;
	flip_R_motor.vpid.actual_speed=0.0f;
	flip_R_motor.vpid.err=0;
	flip_R_motor.vpid.last_err=0;
	flip_R_motor.vpid.err_integration=0;
	flip_R_motor.vpid.P_OUT=0;
	flip_R_motor.vpid.I_OUT=0;
	flip_R_motor.vpid.D_OUT=0;
	flip_R_motor.vpid.PID_OUT=0;
	flip_R_motor.vPID_max=3000;
	flip_L_motor.target_current=0;
	flip_L_motor.actual_current=0;
	flip_L_motor.round_cnt=0.0f;
	flip_L_motor.vpid.target_speed=0;
	flip_L_motor.vpid.actual_speed=0.0f;
	flip_L_motor.vpid.err=0;
	flip_L_motor.vpid.last_err=0;
	flip_L_motor.vpid.err_integration=0;
	flip_L_motor.vpid.P_OUT=0;
	flip_L_motor.vpid.I_OUT=0;
	flip_L_motor.vpid.D_OUT=0;
	flip_L_motor.vpid.PID_OUT=0;
	flip_L_motor.vPID_max=3000;
}

/**
	* @function����:Set_Flip_Speed
	* @brief����:��е������ٶȿ���
	* @param����:��е���ٶ�
	* @retval����ֵ:��
  */
void Set_Flip_Speed(int target_speed)
{
	flip_R_motor.vpid.target_speed = target_speed*19; 
	flip_L_motor.vpid.target_speed = (-target_speed)*19;
}

/**
	* @function����:Vpid_Flip_Realize
	* @brief����:��е�۵���ٶ�pidʵ��
	* @param����:��е�۵���ٶ�pidϵ��
	* @retval����ֵ:��
  */
void Vpid_Flip_Realize(float kp,float ki,float kd)
{
	Vpid_Realize(&flip_R_motor, kp, ki, kd);
	Vpid_Realize(&flip_L_motor, kp, ki, kd);
}

/**
	* @brief  Flip������ֵ������can1��
	* @param void
	* @retval void
	* @attention
	*/
void Set_Flip_Current()
{
	uint8_t current_msg[8];

	//���Ŀ�����Ϊ�ٶ�pid���
	flip_R_motor.target_current = flip_R_motor.vpid.PID_OUT;
	flip_L_motor.target_current = flip_L_motor.vpid.PID_OUT;
	//can����ͨ��Э�飬���յ��˵���飬ע�⣬Flip���������
	current_msg[0] = flip_R_motor.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = flip_R_motor.target_current & 0xff;		//1�ŵ��������8λ
	current_msg[2] = flip_L_motor.target_current >> 8;			//1�ŵ��������8λ
	current_msg[3] = flip_L_motor.target_current & 0xff;		//1�ŵ��������8λ

	//can��������֡
 	CAN1_Send_Flip_Msg(current_msg);
}
/**
	* @brief  Flip����ִ�к���
	* @param void
	* @retval void
	* @attention
	*/
void Flip_Task(MOTOR_MOVE_t flip_move)
{
	switch(flip_move)
	{
		case out:
		{
			Set_Flip_Speed(Flip_Out_Speed);
		}
		break;
		case in:
		{
			Set_Flip_Speed(Flip_In_Speed);
		}
		break;
		case stop:
		{
			Set_Flip_Speed(0);
		}
		break;
		default:
			break;
	}
	Vpid_Realize(&flip_R_motor, Flip_Kp, Flip_Ki, Flip_Kd);
	Vpid_Realize(&flip_L_motor, Flip_Kp, Flip_Ki, Flip_Kd);
	Set_Flip_Current();
}

