#include "Key_Task.h"
#include "BSP_Electric_push.h"

typedef enum
{
	KEY_Check,			//�������
	KEY_Press,			//��������
	KEY_Release,		//�����ͷ�
}KEY_State;

uint8_t Keyboard_Mode=0;//0��ң����ģʽ��1�Ǽ���ģʽ
uint8_t Keyboard_Flag=0;//�жϼ���ģʽ,��ֹһ�ΰ����ظ��ж�
uint8_t BigSource_Mode=0;//����Դ��һ��ȡ��
uint8_t AirSource_Mode=0;
uint8_t GroundSource_Mode=0;
uint8_t ExchangeSource_Mode=0;
uint8_t AirMineral_Mode=0;
int Key_Cnt=0;
extern RC_ctrl_t rc_ctrl;
extern int Key_Vw,Key_Vs,Key_Va,Key_Vd;
KEY_State KeyState = KEY_Check ;
extern uint8_t Flip_PID2_Flag;
int First_Time=0;
int Air_Time=0;
int Ground_Time=0;
int Exchange_Time=0;
int Reset_Time=0;
uint8_t time_mark=0;
uint8_t Air_mark=0;
uint8_t Ground_mark=0;
uint8_t Exchange_mark=0;
uint8_t BigSource_Reset=0;
uint8_t Exchange_Reset=0;
uint8_t Ground_Reset=0;
uint8_t Reset_Mode=0;
uint8_t Reset_mark=0;

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
					KeyState = KEY_Check ;
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
			Keyboard_Mode=1;		//�򿪼���ģʽ
		else if(Keyboard_Mode==1)
			Keyboard_Mode=0;		//�رռ���ģʽ
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
		if((rc_ctrl.mouse.press_l==1) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))//��צ��ȡ
		{
			Clamp_ON;
			Flip_PID2_Flag=1;
		}
		if((rc_ctrl.mouse.press_r==1) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))//��צ�ɿ�
		{
			Clamp_OFF;
			Flip_PID2_Flag=0;
		}
	/*******��ʯ��ת*******/	
		if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_X) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Longitudinal(back);
			Mineral_Task_Widthwise(stop);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_X) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Longitudinal(forward);
			Mineral_Task_Widthwise(stop);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && !(rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Widthwise(right);
			Mineral_Task_Longitudinal(stop);
		}
		else if(((rc_ctrl.key.v & KEY_CTRL)) && !(rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Widthwise(left);
			Mineral_Task_Longitudinal(stop);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Longitudinal(up);
			Mineral_Task_Widthwise(up);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_X) && (rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Longitudinal(down);
			Mineral_Task_Widthwise(down);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Longitudinal(stop);
			Mineral_Task_Widthwise(stop);
			Flip_Task(stop);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_R)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Mineral_Task_Longitudinal(stop);
			Mineral_Task_Widthwise(stop);
			Flip_Task(stop);
		}			/*******��צ��ת*******/
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (rc_ctrl.key.v & KEY_R) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Flip_Task(in);//out
			Mineral_Task_Widthwise(up);
			Mineral_Task_Longitudinal(up);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_X)) && (!(rc_ctrl.key.v & KEY_Z)) && (rc_ctrl.key.v & KEY_R) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Flip_Task(out);//in
			Mineral_Task_Widthwise(stop);
			Mineral_Task_Longitudinal(stop);
		}
		

//	/*******��Ԯצ�����ջ�*******/	
//		if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_Q)))
//		{
//			if(rc_ctrl.key.v & KEY_C)
//			{
//			  RC_Rescue_Move_Send(in);
//			}
//		}
//		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q)))
//		{
//			if(rc_ctrl.key.v & KEY_C)
//			{
//				RC_Rescue_Move_Send(out);
//			}
//			else	RC_Rescue_Move_Send(stop);
//		}
//				
//	/*******��Ԯ������ջ�*******/
//		if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_Q)))
//		{
//			if(rc_ctrl.key.v & KEY_V)
//			{
//				RC_Rescue_card_Send(in);
//			}
//		}
//		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q)))
//		{
//			if(rc_ctrl.key.v & KEY_V)
//			{
//				RC_Rescue_card_Send(out);
//			}
//			else	RC_Rescue_card_Send(stop);
//		}
	
	/*******��е������ջ�*******/
		if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Handle_Task(in);//out
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Handle_Task(out);//in
		}
		else if((!(rc_ctrl.key.v & KEY_E)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Handle_Task(stop);
		}


	/*******���Ƹ�����*******/
		MOTOR_MOVE_t Push_F, Push_B;
		Push_F = stop;
		Push_B = stop;
		/*****ǰ���Ƹ�����*****/
		if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_F) && (!(rc_ctrl.key.v & KEY_G)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_F = qs; Push_B = ht; Push_Task(Push_F, Push_B);
		}
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_F) && (!(rc_ctrl.key.v & KEY_G)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_F = qj; Push_B = ht; Push_Task(Push_F, Push_B);
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_F)) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_B = hs; Push_F = qt; Push_Task(Push_F, Push_B);
		}	
		else if((rc_ctrl.key.v & KEY_CTRL) && (!(rc_ctrl.key.v & KEY_F)) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_B = hj; Push_F = qt; Push_Task(Push_F, Push_B);
		}	
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_F) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_B = hs; Push_F = qs; Push_Task(Push_F, Push_B);
		}	
		else if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_F) && (rc_ctrl.key.v & KEY_G) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_B = hj; Push_F = qj; Push_Task(Push_F, Push_B);
		}	
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_F)) && (!(rc_ctrl.key.v & KEY_G)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
		}	

		/****�ս�ģʽ****/
		if(rc_ctrl.key.v & KEY_CTRL && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
	    if(rc_ctrl.key.v & KEY_B)
      {
				AirMineral_Mode=0;
      }
		}
		else if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
		{
	    if(rc_ctrl.key.v & KEY_B)
      {
				AirMineral_Mode=1;
      }
		}

	/*******һ��ȡ��*******/
	if((rc_ctrl.key.v & KEY_Q)  &&  (!(rc_ctrl.key.v & KEY_CTRL))&& (!(rc_ctrl.key.v & KEY_C)))
	{
		if(BigSource_Mode==0)
			BigSource_Mode=1;		//����Դ��һ��ȡ���
			BigSource_Reset=1;  //��λ��־��
	} 
	if((!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_CTRL))&& (!(rc_ctrl.key.v & KEY_C)))
	{
		if(BigSource_Mode==1)
			BigSource_Mode=0;		//����Դ��һ��ȡ��ر�
		if(time_mark==1)
			time_mark=0;
	} 
	
	if(BigSource_Mode==1)
	{       
		if(time_mark == 0)						//ʱ����ʼλ��¼
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
	
	
	
//
	
//	
//	/*******һ����ȡ���Ͽ�ʯ*******/
//	if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_V)&& (!(rc_ctrl.key.v & KEY_Q)))
//	{
//	  if(GroundSource_Mode==0)
//			GroundSource_Mode=1;		//����ȡ���
//			Ground_Reset=1;
//	} 
//	if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_V))&& (!(rc_ctrl.key.v & KEY_Q)))
//	{
//		if(GroundSource_Mode==1)
//			GroundSource_Mode=0;		//����ȡ��ر�
//		if(Ground_mark==1)
//			Ground_mark=0;
//	}
//	if(GroundSource_Mode==1)
//	{       
//		if(Ground_mark == 0)						//ʱ����ʼλ��¼
//		{
//			Ground_Time = HAL_GetTick();
//			Ground_mark = 1;
//		}
//		if((HAL_GetTick()-Ground_Time)>0 && (HAL_GetTick()-Ground_Time)<4300)
//		{
//			Push_B = hs; Push_F = qt; Push_Task(Push_F, Push_B);
//		}
//		else if((HAL_GetTick()-Ground_Time)>4300 && (HAL_GetTick()-Ground_Time)<4400)
//		{
//			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
//		}
//		if((HAL_GetTick()-Ground_Time)>2500 && (HAL_GetTick()-Ground_Time)<4000)
//		{
//			Handle_Task(in);//out
//		}
//		else if((HAL_GetTick()-Ground_Time)>3950 && (HAL_GetTick()-Ground_Time)<4400)
//		{
//			Handle_Task(stop);//stop
//		}		
//		if((HAL_GetTick()-Ground_Time)>3250 && (HAL_GetTick()-Ground_Time)<4000)
//		{
//			Flip_Task(in);//out
//		}
//		else if((HAL_GetTick()-Ground_Time)>4000 && (HAL_GetTick()-Ground_Time)<20000)
//		{
//			Flip_Task(stop);
//		}
//	}
//	
//	
//	
//	
//		/*******һ���һ�վ*******/   /****���Ƹ˸߶Ȼ�û����****/  /****��Ҫ�ȵ����ÿ�ʯ�������ֶ���ס****/
//	if((!(rc_ctrl.key.v & KEY_CTRL)) && (rc_ctrl.key.v & KEY_C)&& (!(rc_ctrl.key.v & KEY_Q)))
//	{
//	  if(ExchangeSource_Mode==0)
//			{
//				ExchangeSource_Mode=1;		//һ���һ���
//				Exchange_Reset=1;
//			}
//	} 
//	if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_C))&& (!(rc_ctrl.key.v & KEY_Q)))
//	{
//		if(ExchangeSource_Mode==1)
//			ExchangeSource_Mode=0;		//һ���һ��ر�
//		if(Exchange_mark==1)
//			Exchange_mark=0;
//	}
//	if(ExchangeSource_Mode==1)
//	{       
//		if(Exchange_mark == 0)						//ʱ����ʼλ��¼
//		{
//			Exchange_Time = HAL_GetTick();
//			Exchange_mark = 1;
//		}
//		if((HAL_GetTick()-Exchange_Time)>0 && (HAL_GetTick()-Exchange_Time)<4000)
//		{
//			Push_B = hs; Push_F = qs; Push_Task(Push_F, Push_B);
//		}
//		else if((HAL_GetTick()-Exchange_Time)>4000 && (HAL_GetTick()-Exchange_Time)<4100)
//		{
//			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
//		}
//		if((HAL_GetTick()-Exchange_Time)>4000 && (HAL_GetTick()-Exchange_Time)<4750)
//		{
//			Handle_Task(in);//out
//		}
//		else if((HAL_GetTick()-Exchange_Time)>4750 && (HAL_GetTick()-Exchange_Time)<5000)
//		{
//			Handle_Task(stop);//stop
//		}		
//		if((HAL_GetTick()-Exchange_Time)>4000 && (HAL_GetTick()-Exchange_Time)<5800)
//		{
//			Flip_Task(in);//out
//		}
//		else if((HAL_GetTick()-Exchange_Time)>5800 && (HAL_GetTick()-Exchange_Time)<5900)
//		{
//			Flip_Task(stop);
//		}
//	}
//	
//	if((!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_E))&& (!(rc_ctrl.key.v & KEY_C)) && (!(rc_ctrl.key.v & KEY_V)))
//	{
//		Handle_Task(stop);
//	}
//	
//		/*******һ����λ*******/
//		if(((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_Q)) || ((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_C)) || ((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_V)))
//		{
//			if(Reset_Mode==0)
//				Reset_Mode=1;
//		}
//		if((!(rc_ctrl.key.v & KEY_C)) && (!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_V)))
//		{
//			if(Reset_Mode==1)
//				Reset_Mode=0;
//			if(Reset_mark==1)
//				Reset_mark=0;
//		}	
//		if((Reset_Mode==1) && (Reset_mark==0))
//		{
//			Reset_Time = HAL_GetTick();
//			Reset_mark = 1;
//			if(BigSource_Reset==1)
//			{
//				BigSource_Reset=0;
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1900)
//				{
//					Flip_Task(out);//in
//				}
//				else if((HAL_GetTick()-Reset_Time)>1900 && (HAL_GetTick()-Reset_Time)<2000)
//				{
//					Flip_Task(stop);
//				}
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1900)
//				{
//					Handle_Task(out);//in
//				}
//				else if((HAL_GetTick()-Reset_Time)>1900 && (HAL_GetTick()-Reset_Time)<2000)
//				{
//					Handle_Task(stop);//stop
//				}						
//				if((HAL_GetTick()-Reset_Time)>1900 && (HAL_GetTick()-Reset_Time)<4800)
//				{
//					Push_B = hj; Push_F = qj; Push_Task(Push_F, Push_B);
//				}
//				else if((HAL_GetTick()-Reset_Time)>4800 && (HAL_GetTick()-Reset_Time)<4900)
//				{
//					Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
//				}
//			}
//			if(Ground_Reset==1)
//			{
//				Ground_Reset=0;
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Flip_Task(out);//in
//				}
//				else if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Flip_Task(stop);
//				}
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Handle_Task(out);//in
//				}
//				else if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Handle_Task(stop);//stop
//				}						
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Push_B = hj; Push_F = qj; Push_Task(Push_F, Push_B);
//				}
//				else if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
//				}
//			}
//			if(Exchange_Reset==1)
//			{
//				Exchange_Reset=0;
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Flip_Task(out);//in
//				}
//				else if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Flip_Task(stop);
//				}
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Handle_Task(out);//in
//				}
//				else if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Handle_Task(stop);//stop
//				}						
//				if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Push_B = hj; Push_F = qj; Push_Task(Push_F, Push_B);
//				}
//				else if((HAL_GetTick()-Reset_Time)>0 && (HAL_GetTick()-Reset_Time)<1)
//				{
//					Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
//				}
//			}
//		}
		/*******һ��ȡ��ս�*******/

//	if((rc_ctrl.key.v & KEY_CTRL) && (rc_ctrl.key.v & KEY_Q)&& (!(rc_ctrl.key.v & KEY_C)))
//	{
//		if(AirSource_Mode==0)
//			AirSource_Mode=1;		//�ս�һ��ȡ���
//	} 
//	if((!(rc_ctrl.key.v & KEY_CTRL)) && (!(rc_ctrl.key.v & KEY_Q))&& (!(rc_ctrl.key.v & KEY_C)))
//	{
//		if(AirSource_Mode==1)
//			AirSource_Mode=0;		//�ս�һ��ȡ��ر�
//		if(Air_mark==1)
//			Air_mark=0;
//	} 
//	
//	if(AirSource_Mode==1)
//	{       
//		if(Air_mark == 0)						//ʱ����ʼλ��¼
//		{
//			Air_Time = HAL_GetTick();//2200  4300
//			Air_mark = 1; 
//		}
//		if((HAL_GetTick()-Air_Time)>0 && (HAL_GetTick()-Air_Time)<4300)
//		{
//			Push_B = ht; Push_F = qs; Push_Task(Push_F, Push_B);
//		}
//		else if((HAL_GetTick()-Air_Time)>4300 && (HAL_GetTick()-Air_Time)<4400)
//		{
//			Push_B = ht; Push_F = qt; Push_Task(Push_F, Push_B);
//		}
//		if((HAL_GetTick()-Air_Time)>2500 && (HAL_GetTick()-Air_Time)<4000)
//		{
//			Handle_Task(in);//out
//		}
//		else if((HAL_GetTick()-Air_Time)>3950 && (HAL_GetTick()-Air_Time)<4400)
//		{
//			Handle_Task(stop);//stop
//		}		
//		if((HAL_GetTick()-Air_Time)>3250 && (HAL_GetTick()-Air_Time)<4000)
//		{
//			Flip_Task(in);//out
//		}
//		else if((HAL_GetTick()-Air_Time)>4000 && (HAL_GetTick()-Air_Time)<4400)
//		{
//			Flip_Task(stop);
//		}
//	}
//	if(AirMineral_Mode==1)
//	{
//		AirMineral_Task();
//	}

//	
//	
	
 }
}


