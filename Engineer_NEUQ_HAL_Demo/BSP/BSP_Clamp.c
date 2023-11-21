#include "BSP_Clamp.h"

int Clamp_Time=0;
extern uint8_t Flip_PID2_Flag;

void AirMineral_Task()
{
	if(Clamp_Judge==1)
	{
		Clamp_ON;
		Flip_PID2_Flag=1;
	}
	else if(Clamp_Judge==0)
	{
		Clamp_Time= HAL_GetTick();
		if((HAL_GetTick()-Clamp_Time)>=1000)
		{
			if(Clamp_Judge==1)
				Clamp_ON;
			
			else if(Clamp_Judge==0)
				Clamp_OFF;
		}
	}
}

