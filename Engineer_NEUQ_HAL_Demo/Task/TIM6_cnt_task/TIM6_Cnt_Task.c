#include "TIM6_Cnt_Task.h"
#include "BSP_Handle.h"
#include "BSP_Mineral.h"
#include "BSP_Flip.h"
#include "Remote_Task.h"
#include "BSP_Uart.h"
#include "BSP_Chassis.h"
#include "Key_Task.h"
#include "referee.h"
#include "referee_UI.h"

extern TIM_HandleTypeDef htim6;
extern int Chassis_Send_Speed1,Chassis_Send_Speed2,Chassis_Send_Speed3,Chassis_Send_Speed4;
int Tim_cnt=1;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		Tim_cnt++;
			/*****   ң��������    ******/
		Remote_Control();				//ң�������ƴ���		
			/*****   �������    ******/
		Key_Task();
		if(Tim_cnt%7==0)  //ÿ7ms���е����˶�����
		{
			Chassis_Task();
		}
		if(Tim_cnt%100==0)  //ÿ10ms����UI��ʾ
		{
			UI_Display();
		}		
		if(Tim_cnt%37==0)  //ÿ37ms���д��ڷ���
		{
			RC_Chassis_Speed_Send(Chassis_Send_Speed1,Chassis_Send_Speed2,Chassis_Send_Speed3,Chassis_Send_Speed4);
		}
		if(Tim_cnt%10==0)
		{
			referee_unpack_fifo_data();
		}
		if(Tim_cnt>=1000)			//���������־    1s
		Tim_cnt=1;
}
