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
extern unsigned char Longitudinal_Location;			//���λ��
extern unsigned char Lift_Location;							//�ڲ�̧��λ��
extern unsigned char Chuck_Roll_Location;				//��е��roll��λ��
extern unsigned char HP_Zero;										//��Ѫ����־λ
extern unsigned char Out_Lift_Motor_State;			//��̧�����״̬
extern unsigned char Chassis_vx_State;					//����x���˶�״̬
extern unsigned char Chassis_vy_State;					//����y���˶�״̬
extern unsigned char Chassis_wz_State;					//����z���˶�״̬
extern unsigned char Longitudinal_Location_Low;	//����λ�õ�8λ
extern unsigned char Longitudinal_Location_High;//����λ�ø�8λ
extern unsigned char Lift_Location_Low;					//�ڲ�̧��λ�õ�8λ
extern unsigned char Lift_Location_High;				//�ڲ�̧��λ�ø�8λ
extern unsigned char Lift_Location_UI;					//̧����λ��
extern unsigned char Chuck_Roll_Location_Low;		//��е��roll��λ�õ�8λ
extern unsigned char Chuck_Roll_Location_High;	//��е��roll��λ�ø�8λ
extern uint16_t Chuck_Roll_Location_UI;					//����е��roll��ı�־λתΪ360������
extern unsigned char UpperBoard_Data[];					//����̨������������
extern unsigned char Receive_UpperBoard_Data[];	//��̨���ݽ�������
extern unsigned char pUpperBoard_DataH;					//��̨������ʱ��������
extern unsigned char pUpperBoard_DataL;					//��̨������ʱ��������

void Load_UpperBoard_Data(void);
void Resolve_UpperBoard_Data(void);




#endif
