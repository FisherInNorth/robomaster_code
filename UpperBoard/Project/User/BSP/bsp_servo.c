#include "bsp_servo.h"
#include "ChassisBoard.h"

uint8_t angle;
void SERVO_SET_ANGLE(void)
{
	if(View_Servo_State==1)
		angle-=1;
	else if(View_Servo_State==2)
		angle+=1;
	
	if(angle>100)
		angle=100;
	else if(angle<70)
		angle=70;
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

