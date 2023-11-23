#include "Key_Task.h"
#include "BSP_Electric_push.h"

typedef enum
{
	KEY_Check,			//按键检测
	KEY_Press,			//按键按下
	KEY_Release,		//按键释放
}KEY_State;

uint8_t Keyboard_Mode=1;//0是遥控器模式，1是键鼠模式
uint8_t Keyboard_Flag=0;//判断键鼠模式,防止一次按下重复判断
uint8_t BigSource_Mode=0;//大资源岛一键取矿
uint8_t AirSource_Mode=0;
int Key_Cnt=0;
uint8_t AirMineral_Mode=0;
extern RC_ctrl_t rc_ctrl;
extern int Key_Vw,Key_Vs,Key_Va,Key_Vd;
KEY_State KeyState = KEY_Check ;
extern uint8_t Flip_PID2_Flag;
int First_Time=0;
int Air_Time=0;
uint8_t time_mark=0;
uint8_t Air_mark=0;
void Key_Task(void)
{
//	switch(KeyState)
//	{
//		case KEY_Check:
//		{
//			if(rc_ctrl.key.v & KEY_CTRL)
//			{
//				if(rc_ctrl.key.v &KEY_SHIFT)	
//					KeyState=KEY_Press;
//			}
//			break;
//			case KEY_Press:	
//			{
//				if(rc_ctrl.key.v & KEY_CTRL)
//				{
//					if(rc_ctrl.key.v &KEY_SHIFT)
//						KeyState=KEY_Release;
//				}
//			}
//			break;
//			case KEY_Release:
//			{
//				if(rc_ctrl.key.v & KEY_CTRL)
//				{
//					if(rc_ctrl.key.v &KEY_SHIFT)
//						Key_Cnt++;
//				}
//				else 
//				{	
//					Key_Cnt=0;
//					KeyState = KEY_Check ;
//					Keyboard_Flag=1;
//				}
//			}
//			break;
//		}
//	}
//	if(Keyboard_Flag==1)
//	{
//		Keyboard_Flag=0;
//		if(Keyboard_Mode==0)
//			Keyboard_Mode=1;		//打开键鼠模式
//		else if(Keyboard_Mode==1)
//			Keyboard_Mode=0;		//关闭键鼠模式
//	}
	
	if(Keyboard_Mode==1)
	{
		/*******底盘加速模式*******/
		if(rc_ctrl.key.v & KEY_SHIFT)
		{
			if(rc_ctrl.key.v & KEY_W)
			Key_Vw=75; else Key_Vw=0;
			if(rc_ctrl.key.v & KEY_S)
			Key_Vs=-75; else Key_Vs=0;
			if(rc_ctrl.key.v & KEY_A)
			Key_Va=75; else Key_Va=0;
			if(rc_ctrl.key.v & KEY_D)
			Key_Vd=-75; else Key_Vd=0;
		}
		/*******底盘常速模式*******/
		else
		{
			if(rc_ctrl.key.v & KEY_W)
			Key_Vw=440; else Key_Vw=0;
			if(rc_ctrl.key.v & KEY_S)
			Key_Vs=-440; else Key_Vs=0;
			if(rc_ctrl.key.v & KEY_A)
			Key_Va=440; else Key_Va=0;
			if(rc_ctrl.key.v & KEY_D)
			Key_Vd=-440; else Key_Vd=0;

		}	
		/*******夹爪夹取*******/	
		if((rc_ctrl.mouse.press_l==1) && (!(rc_ctrl.key.v & KEY_Q)))//夹爪夹取
		{
			Clamp_ON;
			Flip_PID2_Flag=1;
		}
		if((rc_ctrl.mouse.press_r==1) && (!(rc_ctrl.key.v & KEY_Q)))//夹爪松开
		{
			Clamp_OFF;
			Flip_PID2_Flag=0;
		}
	/*******矿石翻转*******/	
		if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_X) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Longitudinal(back);
			Mineral_Task_Widthwise(stop);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_X) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Longitudinal(forward);
			Mineral_Task_Widthwise(stop);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && !(rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Widthwise(right);
			Mineral_Task_Longitudinal(stop);
		}
		else if(((rc_ctrl.key.v & KEY_CTRL)) && !(rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Widthwise(left);
			Mineral_Task_Longitudinal(stop);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Longitudinal(up);
			Mineral_Task_Widthwise(up);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Longitudinal(down);
			Mineral_Task_Widthwise(down);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Longitudinal(stop);
			Mineral_Task_Widthwise(stop);
			Flip_Task(stop);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Mineral_Task_Longitudinal(stop);
			Mineral_Task_Widthwise(stop);
			Flip_Task(stop);
		}			
		/*******夹爪旋转*******/
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (rc_ctrl.key.v & KEY_R) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Flip_Task(in);//out
			Mineral_Task_Widthwise(up);
			Mineral_Task_Longitudinal(up);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (rc_ctrl.key.v & KEY_R) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Flip_Task(out);//in
			Mineral_Task_Widthwise(stop);
			Mineral_Task_Longitudinal(stop);
		}
		

	/*******救援爪翻出收回*******/	
		if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			if(rc_ctrl.key.v & KEY_C)
			{
			  RC_Rescue_Move_Send(in);
			}
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			if(rc_ctrl.key.v & KEY_C)
			{
				RC_Rescue_Move_Send(out);
			}
			else	RC_Rescue_Move_Send(stop);
		}
				
	/*******救援卡伸出收回*******/
		if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			if(rc_ctrl.key.v & KEY_V)
			{
				RC_Rescue_card_Send(in);
			}
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			if(rc_ctrl.key.v & KEY_V)
			{
				RC_Rescue_card_Send(out);
			}
			else	RC_Rescue_card_Send(stop);
		}
	
	/*******机械臂伸出收回*******/
		if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Handle_Task(in);//out
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Handle_Task(out);//in
		}
		else if((!(rc_ctrl.key.v & KEY_E)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Handle_Task(stop);
		}


	/*******电推杆升降*******/
		MOTOR_MOVE_t Push_F, Push_B;
		Push_F = stop;
		Push_B = stop;
		/*****前电推杆升降*****/
		if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_F) && (!(rc_ctrl.key.v & KEY_G)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_F = qs; Push_B = ht; Push_Task(Push_F, Push_B);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_F) && (!(rc_ctrl.key.v & KEY_G)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_F = qj; Push_B = ht; Push_Task(Push_F, Push_B);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_F)) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_B = hs; Push_F = qt; Push_Task(Push_F, Push_B);
		}	
		else if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_F)) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_B = hj; Push_F = qt; Push_Task(Push_F, Push_B);
		}	
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_F) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_B = hs; Push_F = qs; Push_Task(Push_F, Push_B);
		}	
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_F) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_B = hj; Push_F = qj; Push_Task(Push_F, Push_B);
		}	
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_F)) && (!(rc_ctrl.key.v & KEY_G)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
		}	

		/****空接模式****/
		if(rc_ctrl.key.v & KEY_CTRL && (!(rc_ctrl.key.v & KEY_Q)))
		{
	    if(rc_ctrl.key.v & KEY_B)
      {
				AirMineral_Mode=0;
      }
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q)))
		{
	    if(rc_ctrl.key.v & KEY_B)
      {
				AirMineral_Mode=1;
      }
		}

	/*******一键取矿*******/
	if((rc_ctrl.key.v & KEY_Q)  &&  (!(rc_ctrl.key.v & KEY_CTRL)))
	{
		if(BigSource_Mode==0)
			BigSource_Mode=1;		//大资源岛一键取矿打开
	} 
	if((!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_CTRL)))
	{
		if(BigSource_Mode==1)
			BigSource_Mode=0;		//大资源岛一键取矿关闭
		if(time_mark==1)
			time_mark=0;
	} 
	
	if(BigSource_Mode==1)
	{       
		if(time_mark == 0)						//时间起始位记录
		{
			First_Time = HAL_GetTick();
			time_mark = 1; 
		}
		if((HAL_GetTick()-First_Time)>0 && (HAL_GetTick()-First_Time)<2960)
		{
			Push_B = hs; Push_F = qs; Push_Task(Push_F, Push_B);
		}
		else if((HAL_GetTick()-First_Time)>2960 && (HAL_GetTick()-First_Time)<3050)
		{
			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
		}
		if((HAL_GetTick()-First_Time)>1250 && (HAL_GetTick()-First_Time)<2960)
		{
			Handle_Task(in);//out
		}
		else if((HAL_GetTick()-First_Time)>2960 && (HAL_GetTick()-First_Time)<3050)
		{
			Handle_Task(stop);//stop
		}		
		if((HAL_GetTick()-First_Time)>2210 && (HAL_GetTick()-First_Time)<2960)
		{
			Flip_Task(in);//out
		}
		else if((HAL_GetTick()-First_Time)>2960 && (HAL_GetTick()-First_Time)<3050)
		{
			Flip_Task(stop);
		}
	}
	
	
	
	/*******一键取矿空接*******/
	if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_Q))
	{
		if(AirSource_Mode==0)
			AirSource_Mode=1;		//空接一键取矿打开
	} 
	if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q)))
	{
		if(AirSource_Mode==1)
			AirSource_Mode=0;		//空接一键取矿关闭
		if(Air_mark==1)
			Air_mark=0;
	} 
	
	if(AirSource_Mode==1)
	{       
		if(Air_mark == 0)						//时间起始位记录
		{
			Air_Time = HAL_GetTick();//2200  4300
			Air_mark = 1; 
		}
		if((HAL_GetTick()-Air_Time)>0 && (HAL_GetTick()-Air_Time)<4300)
		{
			Push_B = ht; Push_F = qs; Push_Task(Push_F, Push_B);
		}
		else if((HAL_GetTick()-Air_Time)>4300 && (HAL_GetTick()-Air_Time)<4400)
		{
			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
		}
		if((HAL_GetTick()-Air_Time)>2500 && (HAL_GetTick()-Air_Time)<4000)
		{
			Handle_Task(in);//out
		}
		else if((HAL_GetTick()-Air_Time)>3950 && (HAL_GetTick()-Air_Time)<20000)
		{
			Handle_Task(stop);//stop
		}		
		if((HAL_GetTick()-Air_Time)>3250 && (HAL_GetTick()-Air_Time)<4000)
		{
			Flip_Task(in);//out
		}
		else if((HAL_GetTick()-Air_Time)>4000 && (HAL_GetTick()-Air_Time)<20000)
		{
			Flip_Task(stop);
		}
	}
	if(AirMineral_Mode==1)
	{
		AirMineral_Task();
	}
	if((!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_E)))
	{
		Handle_Task(stop);
	}
 }
}


