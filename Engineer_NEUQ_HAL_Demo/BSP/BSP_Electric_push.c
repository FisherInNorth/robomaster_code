#include "BSP_Electric_push.h"


MOTOR_MOVE_t push_move1, push_move2;
int pushCheck_q= 0;
uint32_t ti_qs = 0; 
int mark_qs = 0;
uint32_t ti_qj = 0; 
int mark_qj = 0;
int pushCheck_h= 0;
uint32_t ti_hs = 0; 
int mark_hs = 0;
uint32_t ti_hj = 0; 
int mark_hj = 0;

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
			if(pushCheck_q == 0)
			{
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,1000);
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,10);
			}
			else if(pushCheck_q == 1)
			{
				if(mark_qs == 0)
				{
					ti_qs = HAL_GetTick();
					mark_qs = 1;
				}
				else if(mark_qs == 1)
				{
					if(HAL_GetTick() - ti_qs > 100)
					{
						pushCheck_q = 0;
						mark_qs = 0;
					}
				}
			}
		}
		break;
		case qj:
		{
			if(pushCheck_q == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,1000);
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,70);
				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,1000);
			}
			else if(pushCheck_q == 0)
			{
				if(mark_qj == 0)
				{
					ti_qj = HAL_GetTick();
					mark_qj = 1;
				}
				else if(mark_qj == 1)
				{
					if(HAL_GetTick() - ti_qj > 100)
					{
						pushCheck_q = 1;
						mark_qj = 0;
					}
				}
			}
			
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
			if(pushCheck_h == 0)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1000);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1000);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
			}
			else if(pushCheck_h == 1)
			{
				if(mark_hs == 0)
				{
					ti_hs = HAL_GetTick();
					mark_hs = 1;
				}
				else if(mark_hs == 1)
				{
					if(HAL_GetTick() - ti_hs > 100)
					{
						pushCheck_h = 0;
						mark_hs = 0;
					}
				}
			}
		}
		break;
		case hj:
		{
			if(pushCheck_h == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1000);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000);
			}
			else if(pushCheck_h == 0)
			{
				if(mark_hj == 0)
				{
					ti_hj = HAL_GetTick();
					mark_hj = 1;
				}
				else if(mark_hj == 1)
				{
					if(HAL_GetTick() - ti_hj > 100)
					{
						pushCheck_h = 1;
						mark_hj = 0;
					}
				}
			}
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
	

