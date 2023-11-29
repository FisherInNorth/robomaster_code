#include "BSP_Rescue_card.h"

MOTOR_t card_motor;
MOTOR_MOVE_t card_move;

/**
	* @function函数:Card_Init
	* @brief描述:复活卡初始化
	* @param输入:无
	* @retval返回值:无
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
	* @function函数:Set_Card_Speed
	* @brief描述:复活卡伸出速度控制
	* @param输入:复活卡速度
	* @retval返回值:无
  */
void Set_Card_Speed(int target_speed)
{
	card_motor.vpid.target_speed = target_speed;
}

/**
	* @function函数:Vpid_Card_Realize
	* @brief描述:复活卡电机速度pid实现
	* @param输入:复活卡电机速度pid系数
	* @retval返回值:无
  */
void Vpid_Card_Realize(float kp,float ki,float kd)
{
		Vpid_Realize(&card_motor,Card_Kp,Card_Ki,Card_Kd);
}

/**
	* @brief  card电流赋值函数（can1）
	* @param void
	* @retval void
	* @attention
	*/
void Set_Card_Current()
{
	uint8_t current_msg[8];

	//电机目标电流为速度pid输出
	card_motor.target_current = card_motor.vpid.PID_OUT;

	//can总线通信协议，参照电调说明书
	current_msg[4] = card_motor.target_current >> 8;
	current_msg[5] = card_motor.target_current & 0xff;

	//can发送数据帧
  CAN1_Card_Msg(current_msg);
}
/**
	* @brief  Card任务执行函数
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
