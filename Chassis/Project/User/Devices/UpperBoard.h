#ifndef __UPPERBOARD_H__
#define __UPPERBOARD_H__

#include "DR16.h"

extern unsigned char Lift_Motor_State;						//̧�����״̬
extern unsigned char Longitudinal_Motor_State;		//�������״̬
extern unsigned char Horizontal_Motor_State;			//���Ƶ��״̬
extern unsigned char Chuck_Yaw_State;		        //Yaw���״̬
extern unsigned char Chuck_Pitch_State;					//Pitcḩ�����״̬
extern unsigned char Chuck_Roll_State;		        //roll���״̬
extern unsigned char PumpAll_State;		        //����״̬
extern unsigned char View_Servo_State;          //ͼ�����,1��ʾ̧��,2��ʾ�½�
extern unsigned char Longitudinal_Ready_State;	//�������һ��ȡ���ջ�ʱ�Ƿ��Ѿ���ȫ�ջ�,0��ʾδ��ȫ�ջ�,1��ʾ����ȫ�ջ�
extern unsigned char Lift_Ready_State;          //̧������Ƿ�ﵽԤ���߶�
extern unsigned char Pump_State;                //����
extern unsigned char ROLL_Ready_State;          //ROLL��ʹ��	
extern unsigned char Longitudinal_Location;
extern unsigned char Lift_Location;
extern unsigned char Chuck_Roll_Location;

extern unsigned char UpperBoard_Data[];
extern unsigned char Receive_UpperBoard_Data[];
extern unsigned char pUpperBoard_DataH;
extern unsigned char pUpperBoard_DataL;

void Load_UpperBoard_Data(void);
void Resolve_UpperBoard_Data(void);




#endif
