#include "BSP_Electric_push.h"


MOTOR_MOVE_t push_move1, push_move2;

/**
	* @brief  电推杆PWM通道函数
	* @param void
	* @retval void
	* @attention
	*/
void TIM_Push_PWM_Init()
{
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	__HAL_TIM_CLEAR_IT(&htim4,TIM_CHANNEL_1);
	__HAL_TIM_CLEAR_IT(&htim4,TIM_CHANNEL_2);
	__HAL_TIM_CLEAR_IT(&htim4,TIM_CHANNEL_3);
	__HAL_TIM_CLEAR_IT(&htim4,TIM_CHANNEL_4);
	__HAL_TIM_CLEAR_IT(&htim5,TIM_CHANNEL_1);
	__HAL_TIM_CLEAR_IT(&htim5,TIM_CHANNEL_2);
	__HAL_TIM_CLEAR_IT(&htim5,TIM_CHANNEL_3);
	__HAL_TIM_CLEAR_IT(&htim5,TIM_CHANNEL_4);
	
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,0);
	
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
}

/**
	* @brief  电推杆任务执行函数
	* @param void
	* @retval void
	* @attention
	*/
void Push_Task(MOTOR_MOVE_t push_moveF, MOTOR_MOVE_t push_moveB)
{
	switch(push_moveF)
	{
		case qs:
		{
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,1000);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,1000);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,10);
		}
		break;
		case qj:
		{
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,1000);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,70);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,1000);
		}
		break;
		case qt:
		{
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,0);
		}			
		break;
		default:
			break;
		}
	switch(push_moveB)
	{
		case hs:
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1000);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1000);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
		}
		break;
		case hj:
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1000);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000);
		}
		break;		
		case ht:
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
		}			
		break;		
		default:
			break;
	}
}
	

