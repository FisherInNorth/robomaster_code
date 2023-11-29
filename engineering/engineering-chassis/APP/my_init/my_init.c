#include "my_init.h"


void all_init(void)
{

	Card_Init();
	Claw_Init();
	Chassis_Init();
  HAL_TIM_Base_Start_IT(&htim6);
	bsp_can1_init();
	bsp_can2_init();


}
