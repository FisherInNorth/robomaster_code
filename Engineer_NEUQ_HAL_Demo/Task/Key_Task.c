#include "Key_Task.h"

uint8_t Keyboard_Mode=0;//0��ң����ģʽ��1�Ǽ���ģʽ
RC_ctrl_t Keyboard;
extern int Key_Vw,Key_Vs,Key_Va,Key_Vd;

void Key_Task(void)
{
	if(Keyboard.key.v & KEY_CTRL)
	{
		if(Keyboard.key.v &KEY_SHIFT)
		{
			Keyboard_Mode=1;		//�򿪼���ģʽ
		}
	}
	if(Keyboard_Mode==1)
	{
		/*******���̼���ģʽ*******/
		if((Keyboard.key.v & KEY_W)==1&&(Keyboard.key.v & KEY_SHIFT)==1)
			Key_Vw=660;
		else Key_Vw=0;
		if((Keyboard.key.v & KEY_S)==1&&(Keyboard.key.v & KEY_SHIFT)==1)
			Key_Vs=-660;
		else Key_Vs=0;
		if((Keyboard.key.v & KEY_W)==1&&(Keyboard.key.v & KEY_SHIFT)==1)
			Key_Va=660;
		else Key_Va=0;
		if((Keyboard.key.v & KEY_S)==1&&(Keyboard.key.v & KEY_SHIFT)==1)
			Key_Vd=-660;
		else Key_Vd=0;
		/*******���̳���ģʽ*******/
		if((Keyboard.key.v & KEY_W)==1&&(Keyboard.key.v & KEY_SHIFT)==0)
			Key_Vw=440;
		else Key_Vw=0;
		if((Keyboard.key.v & KEY_S)==1&&(Keyboard.key.v & KEY_SHIFT)==0)
			Key_Vs=-440;
		else Key_Vs=0;
		if((Keyboard.key.v & KEY_W)==1&&(Keyboard.key.v & KEY_SHIFT)==0)
			Key_Va=440;
		else Key_Va=0;
		if((Keyboard.key.v & KEY_S)==1&&(Keyboard.key.v & KEY_SHIFT)==0)
			Key_Vd=-440;
		else Key_Vd=0;		
	
		/*******��צ��ȡ*******/	
	//if(Keyboard.mouse.press_l==1)��צ��ȡ
	//if(Keyboard.mouse.press_R==1)��צ�ɿ�

	/*******��ʯ��ת*******/	
	if((Keyboard.key.v & KEY_Z)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
		mineral_widthwise_move=right;
	else if((Keyboard.key.v & KEY_Z)==1&&(Keyboard.key.v & KEY_CTRL)==1)
		mineral_widthwise_move=left;
	if((Keyboard.key.v & KEY_X)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
		mineral_longitudinal_move=forward;
	else if((Keyboard.key.v & KEY_X)==1&&(Keyboard.key.v & KEY_CTRL)==1)
		mineral_longitudinal_move=back;
	
	/*******��ʯ����*******/	
	if((Keyboard.key.v & KEY_Z)==1&&(Keyboard.key.v & KEY_X)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ mineral_widthwise_move=up; mineral_longitudinal_move=up; }
	else if((Keyboard.key.v & KEY_Z)==1&&(Keyboard.key.v & KEY_X)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ mineral_widthwise_move=down; mineral_longitudinal_move=down; }
	
	/*******��Ԯצ����ջ�*******/	
	if((Keyboard.key.v & KEY_C)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ rescue_move=out; }
	else if((Keyboard.key.v & KEY_C)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ rescue_move=in; }
	else rescue_move=stop;
	
	/*******��Ԯ������ջ�*******/
	if((Keyboard.key.v & KEY_V)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ rescue_card=out; }
	else if((Keyboard.key.v & KEY_V)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ rescue_card=in; }
	else rescue_card=stop;
	
	/*******��צ����ջ�*******/
	if((Keyboard.key.v & KEY_E)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ handle_move=out; }
	else if((Keyboard.key.v & KEY_E)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ handle_move=in; }
	else handle_move=stop;
	
	/*******���Ƹ�����*******/
	 /*****ǰ���Ƹ�����*****/
	if((Keyboard.key.v & KEY_F)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ push1=out; }
	else if((Keyboard.key.v & KEY_F)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ push1=in; }
	 /*****ǰ���Ƹ�����*****/
	if((Keyboard.key.v & KEY_G)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ push2=out; }
	else if((Keyboard.key.v & KEY_G)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ push2=in; }
	
	/*******һ��ȡ��*******/
	if((Keyboard.key.v & KEY_G)==1)
	{       }
	
	/*******��צ��ת*******/
	if((Keyboard.key.v & KEY_R)==1&&(Keyboard.key.v & KEY_CTRL)==0)	
	{ steering_move=out; }
	else if((Keyboard.key.v & KEY_R)==1&&(Keyboard.key.v & KEY_CTRL)==1)	
	{ steering_move=in; }
	else steering_move=stop;
	
 }
}


