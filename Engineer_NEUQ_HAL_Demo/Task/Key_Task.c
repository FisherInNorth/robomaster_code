#include "Key_Task.h"

uint8_t Keyboard_Mode=0;//0是遥控器模式，1是键鼠模式
extern RC_ctrl_t rc_ctrl;
extern int Key_Vw,Key_Vs,Key_Va,Key_Vd;

void Key_Task(void)
{
	if(rc_ctrl.key.v & KEY_CTRL)
	{
		if(rc_ctrl.key.v &KEY_SHIFT)
		{
			Keyboard_Mode=1;		//打开键鼠模式
		}
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
	//if(rc_ctrl.mouse.press_l==1)夹爪夹取
	//if(rc_ctrl.mouse.press_R==1)夹爪松开

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
				rescue_move=out;
			}
		}
		else
		{
			if(rc_ctrl.key.v & KEY_C)
			{
				rescue_move=in;
			}
			else rescue_move=stop;
		}
				
	/*******救援卡伸出收回*******/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_C)
			{
				rescue_move=out;
			}
		}
		else
		{
			if(rc_ctrl.key.v & KEY_C)
			{
				rescue_move=in;
			}
			else rescue_move=stop;
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
	 /*****前电推杆升降*****/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_F)
			{
				RC_Push_F_Send(in);
			}
			else RC_Push_F_Send(stop);
		}
		else
		{
			if(rc_ctrl.key.v & KEY_F)
			{
				RC_Push_F_Send(out);
			}
			else RC_Push_F_Send(stop);
		}
	 /*****后电推杆升降*****/
		if(rc_ctrl.key.v & KEY_CTRL)
		{
			if(rc_ctrl.key.v & KEY_G)
			{
				RC_Push_B_Send(in);
			}
			else RC_Push_B_Send(stop);
		}
		else
		{
			if(rc_ctrl.key.v & KEY_G)
			{
				RC_Push_B_Send(out);
			}
			else RC_Push_B_Send(stop);
		}
	
	/*******一键取矿*******/ 
	if(rc_ctrl.key.v & KEY_Q)
	{       
		
		
	}
 }
}


