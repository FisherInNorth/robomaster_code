#include "BSP_Handle.h"
#include "BSP_Motor.h"
#include "pid.h"
#include "BSP_Can.h"
MOTOR_MOVE_t handle_move;
MOTOR_t handle_motor;
/**
	* @function����:Handle_Init
	* @brief����:��е�۳�ʼ��
	* @param����:��
	* @retval����ֵ:��
  */
void Handle_Init(void)
{
	handle_motor.target_current=0;
	handle_motor.actual_current=0;
	handle_motor.round_cnt=0.0f;
	handle_motor.vpid.target_speed=0;
	handle_motor.vpid.actual_speed=0.0f;
	handle_motor.vpid.err=0;
	handle_motor.vpid.last_err=0;
	handle_motor.vpid.err_integration=0;
	handle_motor.vpid.P_OUT=0;
	handle_motor.vpid.I_OUT=0;
	handle_motor.vpid.D_OUT=0;
	handle_motor.vpid.PID_OUT=0;
}

/**
	* @function����:Set_Handle_Speed
	* @brief����:��е������ٶȿ���
	* @param����:��е���ٶ�
	* @retval����ֵ:��
  */
void Set_Handle_Speed(int target_speed)
{
	handle_motor.vpid.target_speed = target_speed; 
}

/**
	* @function����:Vpid_Handle_Realize
	* @brief����:��е�۵���ٶ�pidʵ��
	* @param����:��е�۵���ٶ�pidϵ��
	* @retval����ֵ:��
  */
void Vpid_Handle_Realize(float kp,float ki,float kd)
{
	Vpid_Realize(handle_motor, kp, ki, kd);
}

/**
	* @brief  handle������ֵ������can1��
	* @param void
	* @retval void
	* @attention
	*/
void Set_Handle_Current()
{
	uint8_t current_msg[8];

	//���Ŀ�����Ϊ�ٶ�pid���
	handle_motor.target_current = handle_motor.vpid.PID_OUT;

	//can����ͨ��Э�飬���յ��˵����
	current_msg[0] = handle_motor.target_current >> 8;			//1�ŵ��������8λ
	current_msg[1] = handle_motor.target_current & 0xff;		//1�ŵ��������8λ

	//can��������֡
	CAN1_Send_Handle_Msg(current_msg);
}
/**
	* @brief  handle����ִ�к���
	* @param void
	* @retval void
	* @attention
	*/
void Handle_Task(MOTOR_MOVE_t handle_move)
{
	switch(handle_move)
	{
		case out:
		{
			Set_Handle_Speed(Handle_Target_Speed);
		}
		break;
		case in:
		{
			Set_Handle_Speed(-Handle_Target_Speed);
		}
		break;
		case stop:
		{
			Set_Handle_Speed(0);
		}
		break;
		default:
			break;
	}
	
	Vpid_Handle_Realize(Handle_Kp, Handle_Ki, Handle_Kd);
	
	Set_Handle_Current();
}

