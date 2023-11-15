#include "BSP_Clamp.h"

int Clamp_Time=0;

void AirMineral_Task()
{
	if(Clamp_Judge==1)
	{
		Clamp_ON;
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

