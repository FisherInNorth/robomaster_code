#include "BSP_Rescue_card.h"

MOTOR_t card_motor;
MOTOR_MOVE_t card_move;

/**
	* @function����:Card_Init
	* @brief����:�����ʼ��
	* @param����:��
	* @retval����ֵ:��
  */
void Card_Init(void)
{
	card_motor.round_cnt=0.0f;
	card_motor.vpid.target_speed=0;
	card_motor.vpid.actual_speed=0.0f;
	card_motor.target_current=0;
	card_motor.actual_current=0;
	card.vPID_max = 3000;
}

/**
	* @function����:Set_Card_Speed
	* @brief����:�������ٶȿ���
	* @param����:����ٶ�
	* @retval����ֵ:��
  */
void Set_Card_Speed(int target_speed)
{
	card_motor.vpid.target_speed = target_speed;
}

/**
	* @function����:Vpid_Card_Realize
	* @brief����:�������ٶ�pidʵ��
	* @param����:�������ٶ�pidϵ��
	* @retval����ֵ:��
  */
void Vpid_Card_Realize(float kp,float ki,float kd)
{
		Vpid_Realize(&card_motor,Card_Kp,Card_Ki,Card_Kd);
}

/**
	* @brief  card������ֵ������can1��
	* @param void
	* @retval void
	* @attention
	*/
void Set_Card_Current()
{
	uint8_t current_msg[8];

	//���Ŀ�����Ϊ�ٶ�pid���
	card_motor.target_current = card_motor.vpid.PID_OUT;

	//can����ͨ��Э�飬���յ��˵����
	current_msg[4] = card_motor.target_current >> 8;
	current_msg[5] = card_motor.target_current & 0xff;

	//can��������֡
  CAN1_Card_Msg(current_msg);
}
/**
	* @brief  Card����ִ�к���
	* @param void
	* @retval void
	* @attention
	*/
void Card_Task(MOTOR_MOVE_t card_move)
{
	switch(card_move)
	{
		case out:
		{
			Set_Card_Speed(Card_Target_Speed);
		}
		break;
		case in:
		{
			Set_Card_Speed(-Card_Target_Speed);
		}
		break;
		case stop:
		{
			Set_Card_Speed(0);
		}
		break;
		default:
			break;
	}
	
	Vpid_Card_Realize(Card_Kp, Card_Ki, Card_Kd);
	
	Set_Card_Current();
}
