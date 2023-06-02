#include "UpperBoard.h"

/*
0:��ֹ
1:����΢��
2:����΢��
3:��������˶�
4:��������˶�
...(���֧��16��״̬)...
*/

/*
���͸�ʽ��
ÿ���ֽ��Ƕ���������֡
ǰ4λ�ǲ���ID�������������ĸ����������ɿ���16�ֲ���
��4λ��״̬�룬���ò�������ʲô״̬����������16��״̬
*/

unsigned char Lift_Motor_State=0;						//̧�����״̬
unsigned char Longitudinal_Motor_State=0;		//�������״̬
unsigned char Horizontal_Motor_State=0;			//���Ƶ��״̬
unsigned char Chuck_Yaw_State=0;		        //Yaw���״̬
unsigned char Chuck_Pitch_State=0;					//Pitcḩ�����״̬
unsigned char Chuck_Roll_State=0;		        //roll���״̬
unsigned char PumpAll_State=0;		        //����״̬
unsigned char View_Servo_State=0;          //ͼ�����,1��ʾ̧��,2��ʾ�½�
unsigned char Longitudinal_Ready_State=0;	//�������һ��ȡ���ջ�ʱ�Ƿ��Ѿ���ȫ�ջ�,0��ʾδ��ȫ�ջ�,1��ʾ����ȫ�ջ�
unsigned char Lift_Ready_State = 0;				//̧������Ƿ�̧����ָ��λ��
unsigned char Pump_State=0;
unsigned char ROLL_Ready_State=0;
unsigned char Longitudinal_Location=0;
unsigned char Lift_Location=0;
unsigned char Chuck_Roll_Location=0;
unsigned char HP_Zero=0;

unsigned char UpperBoard_Data[16] = {0};
unsigned char Receive_UpperBoard_Data[100] = {0};
unsigned char pUpperBoard_DataH = 0;
unsigned char pUpperBoard_DataL = 0;

unsigned char target_UpperBoard_Data;


extern RC_ctrl_t rc_ctrl;

void Load_UpperBoard_Data(void)
{
	UpperBoard_Data[0] = 0x01<<4;
	UpperBoard_Data[0] |= Lift_Motor_State;
	
	UpperBoard_Data[1] = 0x02<<4;
	UpperBoard_Data[1] |= Longitudinal_Motor_State;
	
	UpperBoard_Data[2] = 0x03<<4;
	UpperBoard_Data[2] |= Horizontal_Motor_State;
	
	UpperBoard_Data[3] = 0x04<<4;
	UpperBoard_Data[3] |= Chuck_Yaw_State;
	
	UpperBoard_Data[4] = 0x05<<4;
	UpperBoard_Data[4] |= Chuck_Pitch_State;
	
	UpperBoard_Data[5] = 0x06<<4;
	UpperBoard_Data[5] |= Chuck_Roll_State;
	
	UpperBoard_Data[6] = 0x07<<4;
	UpperBoard_Data[6] |= View_Servo_State;
	
	UpperBoard_Data[7] = 0x08<<4;
	UpperBoard_Data[7] |= HP_Zero;
}

void Resolve_UpperBoard_Data(void)
{
	while(pUpperBoard_DataL != pUpperBoard_DataH)
	{
		pUpperBoard_DataL ++;
		if(pUpperBoard_DataL == 100)
		{
			pUpperBoard_DataL = 0;
		}
		
		target_UpperBoard_Data = Receive_UpperBoard_Data[pUpperBoard_DataL];
		
		switch(target_UpperBoard_Data >> 4)
		{
			case 0x0001:	Longitudinal_Ready_State = target_UpperBoard_Data & 0x0f; break;
			case 0x0002:  Lift_Ready_State				 = target_UpperBoard_Data & 0x0f; break;
			case 0x0003:  ROLL_Ready_State				 = target_UpperBoard_Data & 0x0f; break;
			case 0x0004:  Longitudinal_Location 	 = target_UpperBoard_Data & 0x0f; break;
			case 0x0005:  Lift_Location						 = target_UpperBoard_Data & 0x0f; break;
			case 0x0006:  Chuck_Roll_Location			 = target_UpperBoard_Data & 0x0f; break;

			//...(���֧��ʮ����)...
			default:break;
		}
	}
}