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
			/*****   遥控器控制    ******/
		Remote_Control();				//遥控器控制代码		
			/*****   键鼠控制    ******/
		Key_Task();
		if(Tim_cnt%7==0)  //每7ms进行底盘运动解算
		{
			Chassis_Task();
		}
		if(Tim_cnt%100==0)  //每10ms进行UI显示
		{
			UI_Display();
		}		
		if(Tim_cnt%37==0)  //每37ms进行串口发送
		{
			RC_Chassis_Speed_Send(Chassis_Send_Speed1,Chassis_Send_Speed2,Chassis_Send_Speed3,Chassis_Send_Speed4);
		}
		if(Tim_cnt%10==0)
		{
			referee_unpack_fifo_data();
		}
		if(Tim_cnt>=1000)			//清除计数标志    1s
		Tim_cnt=1;
}
