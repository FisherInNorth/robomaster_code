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
  * @brief  设置夹爪旋转角度
  * @param  angle: 角度，（0 到 180）
  * @retval 无
	* @attention
  */
void set_steering_gear_angle(uint16_t angle_temp)
{
	angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * htim2.Init.Period;    // 计算角度对应的占空比
	set_steering_gear_dutyfactor(angle_temp);    // 设置占空比
}

/**
  * @brief  设置夹爪旋转占空比
  * @param  dutyfactor: 占空比，（0.5/20.0*htim2.Init.Period 到 2.5/20.0*htim2.Init.Period）
  * @retval 无
  * @attention	定时器2通道1
  */
void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0.5 / 20.0 * htim2.Init.Period > dutyfactor ? 0.5 / 20.0 * htim2.Init.Period : dutyfactor;
	dutyfactor = 2.5 / 20.0 * htim2.Init.Period < dutyfactor ? 2.5 / 20.0 * htim2.Init.Period : dutyfactor;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,dutyfactor);
}

/**
  * @brief  夹爪旋转任务执行函数
  * @param  steering_move
  * @retval 无
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
