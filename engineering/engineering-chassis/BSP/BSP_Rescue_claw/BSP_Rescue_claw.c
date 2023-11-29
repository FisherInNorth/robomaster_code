#include "BSP_Rescue_claw.h"

MOTOR_t claw_L_motor,claw_R_motor;
MOTOR_MOVE_t claw_move;

/**
	* @function����:Claw_Init
	* @brief����:��Ԯצ��ʼ��
	* @param����:��
	* @retval����ֵ:��
  */
void Claw_Init(void)
{
	claw_L_motor.round_cnt=0.0f;
	claw_L_motor.vpid.target_speed=0;
	claw_L_motor.vpid.actual_speed=0.0f;
	claw_L_motor.target_current=0;
	claw_L_motor.actual_current=0;
	claw_R_motor.round_cnt=0.0f;
	claw_R_motor.vpid.target_speed=0;
	claw_R_motor.vpid.actual_speed=0.0f;
	claw_R_motor.target_current=0;
	claw_R_motor.actual_current=0;
	claw_L_motor.vPID_max = 3000;
	claw_R_motor.vPID_max = 3000;
}


/**
	* @function����:Set_Claw_Speed
	* @brief����:��Ԯצ�ٶȿ���
	* @param����:��Ԯצ�ٶ�
	* @retval����ֵ:��
  */
void Set_Claw_Speed(int target_L_speed,int target_R_speed)
{
	claw_L_motor.vpid.target_speed = target_L_speed;
	claw_R_motor.vpid.target_speed = target_R_speed;
}

/**
	* @function����:Vpid_Claw_Realize
	* @brief����:��Ԯצ����ٶ�pidʵ��
	* @param����:��Ԯצ����ٶ�pidϵ��
	* @retval����ֵ:��
  */
void Vpid_Claw_Realize(float kp,float ki,float kd)
{
	Vpid_Realize(&claw_L_motor, Claw_Kp, Claw_Ki, Claw_Kd);
	Vpid_Realize(&claw_R_motor, Claw_Kp, Claw_Ki, Claw_Kd);

}

/**
	* @brief  Claw������ֵ������can1��
	* @param void
	* @retval void
	* @attention
	*/
void Set_Claw_Current()
{
	uint8_t current_msg[8];

	claw_L_motor.target_current = claw_L_motor.vpid.PID_OUT;
	claw_R_motor.target_current = claw_R_motor.vpid.PID_OUT;

	current_msg[0] = claw_L_motor.target_current >> 8;
	current_msg[1] = claw_L_motor.target_current & 0xff;
	current_msg[2] = claw_R_motor.target_current >> 8;
	current_msg[3] = claw_R_motor.target_current & 0xff;
	 //can��������֡
  CAN1_Claw_Msg(current_msg);
}


/**
	* @brief  Claw����ִ�к���
	* @param void
	* @retval void
	* @attention
	*/
void Claw_Task(MOTOR_MOVE_t Claw_move)
{
	switch(Claw_move)
	{
		case out:
		{
			Set_Claw_Speed(Claw_Target_Speed,-Claw_Target_Speed);
		}
		break;
		case in:
		{
			Set_Claw_Speed(-Claw_Target_Speed,Claw_Target_Speed);
		}
		break;
		case stop:
		{
			Set_Claw_Speed(0,0);
		}
		break;
		default:
			break;
	}
	
	Vpid_Claw_Realize(Claw_Kp, Claw_Ki, Claw_Kd);
	
	Set_Claw_Current();
}












