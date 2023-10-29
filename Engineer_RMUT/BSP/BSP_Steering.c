#include "BSP_Steering.h"

MOTOR_MOVE_t steering_move;
int steering_angle = 0;

uint16_t angle_temp;
uint16_t dutyfactor;
void TIM2_PWM_Init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	__HAL_TIM_CLEAR_IT(&htim2,TIM_CHANNEL_1);
	set_steering_gear_angle(0);
}

/**
  * @brief  ���ü�צ��ת�Ƕ�
  * @param  angle: �Ƕȣ���0 �� 180��
  * @retval ��
	* @attention
  */
void set_steering_gear_angle(uint16_t angle_temp)
{
	angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * htim2.Init.Period;    // ����Ƕȶ�Ӧ��ռ�ձ�
	set_steering_gear_dutyfactor(angle_temp);    // ����ռ�ձ�
}

/**
  * @brief  ���ü�צ��תռ�ձ�
  * @param  dutyfactor: ռ�ձȣ���0.5/20.0*htim2.Init.Period �� 2.5/20.0*htim2.Init.Period��
  * @retval ��
  * @attention	��ʱ��2ͨ��1
  */
void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
	/* �Գ�����Χ��ռ�ձȽ��б߽紦�� */
	dutyfactor = 0.5 / 20.0 * htim2.Init.Period > dutyfactor ? 0.5 / 20.0 * htim2.Init.Period : dutyfactor;
	dutyfactor = 2.5 / 20.0 * htim2.Init.Period < dutyfactor ? 2.5 / 20.0 * htim2.Init.Period : dutyfactor;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,dutyfactor);
}

/**
  * @brief  ��צ��ת����ִ�к���
  * @param  steering_move
  * @retval ��
	* @attention
  */
void Steering_Task(MOTOR_MOVE_t steering_move)
{
	switch(steering_move)
	{
		case out:
		{
		if(steering_angle<=102)
    {steering_angle++;}
		else
		{steering_angle = 102;}
		set_steering_gear_angle(steering_angle);
		}
		break;
		case in:
		{
		if(steering_angle > 0)
		{steering_angle--;}
		else
		{steering_angle = 0;}
		set_steering_gear_angle(steering_angle);
		}
		break;
		case stop:
		{
		steering_angle = steering_angle;
		set_steering_gear_angle(steering_angle);
	  }
		break;
		default:
			break;
	}

}
