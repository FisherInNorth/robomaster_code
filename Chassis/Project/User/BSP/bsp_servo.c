#include "bsp_servo.h"


void SERVO_SET_ANGLE(uint8_t angle)
{
	if(angle>40)
		angle=40;
	else if(angle<0)
		angle=0;
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((11.1111f*angle)+500));
	
}



uint8_t tim_counter;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim1)
	{
		tim_counter++;
		if(tim_counter%2==0)
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	}
}

