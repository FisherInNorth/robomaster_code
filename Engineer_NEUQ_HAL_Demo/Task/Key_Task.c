#include "Key_Task.h"
#include "BSP_Electric_push.h"

typedef enum
{
	KEY_Check,			//按键检测
	KEY_Press,			//按键按下
	KEY_Release,		//按键释放
}KEY_State;

uint8_t Keyboard_Mode=0;//0是遥控器模式，1是键鼠模式
uint8_t Keyboard_Flag=0;//判断键鼠模式,防止一次按下重复判断
int Key_Cnt=0;
uint8_t AirMineral_Mode=0;
extern RC_ctrl_t rc_ctrl;
extern int Key_Vw,Key_Vs,Key_Va,Key_Vd;
KEY_State KeyState = KEY_Check ;

void Key_Task(void)
{
	switch(KeyState)
	{
		case KEY_Check:
		{
			if(rc_ctrl.key.v & KEY_CTRL)
			{
				if(rc_ctrl.key.v &KEY_SHIFT)	
					KeyState=KEY_Press;
			}
			break;
			case KEY_Press:	
			{
				if(rc_ctrl.key.v & KEY_CTRL)
				{
					if(rc_ctrl.key.v &KEY_SHIFT)
						KeyState=KEY_Release;
				}
			}
			break;
			case KEY_Release:
			{
				if(rc_ctrl.key.v & KEY_CTRL)
				{
					if(rc_ctrl.key.v &KEY_SHIFT)
						Key_Cnt++;
				}
				else 
				{	
					Key_Cnt=0;
					KeyState = KEY_Press ;
					Keyboard_Flag=1;
				}
			}
			break;
		}
	}
	if(Keyboard_Flag==1)
	{
		Keyboard_Flag=0;
		if(Keyboard_Mode==0)
			Keyboard_Mode=1;		//打开键鼠模式
		else if(Keyboard_Mode==1)
			Keyboard_Mode=0;		//关闭键鼠模式
	}
	if(Keyboard_Mode==1)
	{
		/*******底盘加速模式*******/
		if(rc_ctrl.key.v & KEY_SHIFT)
		{
			if(rc_ctrl.key.v & KEY_W)
			Key_Vw=660; else Key_Vw=0;
			if(rc_ctrl.key.v & KEY_S)
			Key_Vs=-660; else Key_Vs=0;
			if(rc_ctrl.key.v & KEY_A)
			Key_Va=660; else Key_Va=0;
			if(rc_ctrl.key.v & KEY_D)
			Key_Vd=-660; else Key_Vd=0;
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
		if(rc_ctrl.mouse.press_l==1)//夹爪夹取
			Clamp_ON;
		if(rc_ctrl.mouse.press_r==1)//夹爪松开
			Clamp_OFF;
	/*******矿石翻转*******/	
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_Z)
			mineral_widthwise_move=left;
			else mineral_widthwise_move=stop;
			if(rc_ctrl.key.v & KEY_X)
			mineral_longitudinal_move=back;
			else mineral_longitudinal_move=stop;
		}
		else
		{
			if(rc_ctrl.key.v & KEY_Z)
			mineral_widthwise_move=right;
			else mineral_widthwise_move=stop;			
			if(rc_ctrl.key.v & KEY_X)
			mineral_longitudinal_move=forward;
			else mineral_longitudinal_move=stop;
		}

	/*******矿石升降*******/	
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if((rc_ctrl.key.v & KEY_Z)==1)
			{
				if(rc_ctrl.key.v & KEY_X)
				{
					mineral_widthwise_move=up; 
					mineral_longitudinal_move=up;
				}
			}
		}
		else
		{
			if(rc_ctrl.key.v & KEY_Z)
			{
				if(rc_ctrl.key.v & KEY_X)
				{
					mineral_widthwise_move=down; 
					mineral_longitudinal_move=down;
				}
			}
		}
		
	/*******救援爪伸出收回*******/	
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_C)
			{
			  RC_Rescue_Move_Send(in);
			}
		}
		else
		{
			if(rc_ctrl.key.v & KEY_C)
			{
				RC_Rescue_Move_Send(out);
			}
			else	RC_Rescue_Move_Send(stop);
		}
				
	/*******救援卡伸出收回*******/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_V)
			{
				RC_Rescue_card_Send(in);
			}
		}
		else
		{
			if(rc_ctrl.key.v & KEY_V)
			{
				RC_Rescue_card_Send(out);
			}
			else	RC_Rescue_card_Send(stop);
		}
	
	/*******机械臂伸出收回*******/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_E)
			{
				Handle_Task(out);
			}
			else Handle_Task(stop);
		}
		else
		{
			if(rc_ctrl.key.v & KEY_E)
			{
				Handle_Task(in);
			}
			else Handle_Task(stop);
		}
		/*******夹爪旋转*******/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_R)
			{
				Flip_Task(out);
			}
			else Flip_Task(stop);
		}
		else
		{
			if(rc_ctrl.key.v & KEY_R)
			{
				Flip_Task(in);
			}
			else Flip_Task(stop);
		}
	/*******电推杆升降*******/
		MOTOR_MOVE_t Push_F, Push_B;
		Push_F = stop;
		Push_B = stop;
		/*****前电推杆升降*****/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_F)
			{
				Push_F = in;
			}
			else Push_F = stop;
		}
		else
		{
			if(rc_ctrl.key.v & KEY_F)
			{
				Push_F = out;
			}
			else Push_F = stop;
		}
	 /*****后电推杆升降*****/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_G)
			{
				Push_B = in;
			}
			else Push_B = stop;
		}
		else
		{
			if(rc_ctrl.key.v & KEY_G)
			{
				Push_B = out; 
			}
			else Push_B = stop;
		}
	Push_Task(Push_F, Push_B);
		
		/****空接模式****/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
	    if(rc_ctrl.key.v & KEY_B)
      {
				AirMineral_Mode=0;
      }
		}
		else 
		{
	    if(rc_ctrl.key.v & KEY_B)
      {
				AirMineral_Mode=1;
      }
		}
		if(AirMineral_Mode==1)
		{
			if(Clamp_Judge)
			{
				Clamp_ON;
			}
	    else Clamp_OFF;
		}
	/*******一键取矿*******/ 
	if(rc_ctrl.key.v & KEY_Q)
	{       
		
		
	}
 }
}


