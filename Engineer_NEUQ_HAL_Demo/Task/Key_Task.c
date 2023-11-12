#include "Key_Task.h"

uint8_t Keyboard_Mode=0;//0��ң����ģʽ��1�Ǽ���ģʽ
extern RC_ctrl_t rc_ctrl;
extern int Key_Vw,Key_Vs,Key_Va,Key_Vd;

void Key_Task(void)
{
	if(rc_ctrl.key.v & KEY_CTRL)
	{
		if(rc_ctrl.key.v &KEY_SHIFT)
		{
			Keyboard_Mode=1;		//�򿪼���ģʽ
		}
	}
	if(Keyboard_Mode==1)
	{
		/*******���̼���ģʽ*******/
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
		/*******���̳���ģʽ*******/
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
	
		/*******��צ��ȡ*******/	
	//if(rc_ctrl.mouse.press_l==1)��צ��ȡ
	//if(rc_ctrl.mouse.press_R==1)��צ�ɿ�

	/*******��ʯ��ת*******/	
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

	/*******��ʯ����*******/	
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
		
	/*******��Ԯצ����ջ�*******/	
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
				
	/*******��Ԯ������ջ�*******/
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
	
	/*******��е������ջ�*******/
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
		/*******��צ��ת*******/
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
	/*******���Ƹ�����*******/
	 /*****ǰ���Ƹ�����*****/
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
	 /*****����Ƹ�����*****/
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
	
	/*******һ��ȡ��*******/ 
	if(rc_ctrl.key.v & KEY_Q)
	{       
		
		
	}
 }
}


