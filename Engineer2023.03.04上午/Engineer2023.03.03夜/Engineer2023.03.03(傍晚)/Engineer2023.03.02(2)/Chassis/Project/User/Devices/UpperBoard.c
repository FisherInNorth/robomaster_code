#include "UpperBoard.h"

/*
ģʽһ���󲦸˿���Ϊ��ʱ
1.��ҡ�˿���ǰ�������˶�
2.��ҡ�˿�������ת��

ģʽ�����󲦸˿���Ϊ��ʱ
1.��ҡ�˿��ƴ�����¡�ǰ���˶�
2.��ҡ�˿���С��

*/

/*
0:��ֹ
1:����΢��
2:����΢��
3:��������˶�
4:��������˶�
*/
unsigned char Lift_Motor_State;						//̧�����״̬
unsigned char Longitudinal_Motor_State;		//�������״̬
unsigned char Horizontal_Motor_State;     //�����������״̬

unsigned char UpperBoard_Data[8] = {0,0,0,0,0,0,0,0};

void Load_UpperBoard_Data(void)
{
	UpperBoard_Data[0] = 0;
	UpperBoard_Data[0] |= Lift_Motor_State;
	UpperBoard_Data[0] |= Longitudinal_Motor_State<<4;
	UpperBoard_Data[1] = 0;
	UpperBoard_Data[1] |= Horizontal_Motor_State;
}

