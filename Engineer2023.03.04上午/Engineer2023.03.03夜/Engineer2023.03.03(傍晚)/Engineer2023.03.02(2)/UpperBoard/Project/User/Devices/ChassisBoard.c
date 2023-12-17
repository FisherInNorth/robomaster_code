#include "ChassisBoard.h"

unsigned char Lift_Motor_State;						//̧�����״̬
unsigned char Longitudinal_Motor_State;		//�������״̬
unsigned char Horizontal_Motor_State;     //�����������״̬

unsigned char ChassisBoard_Data[8] = {0,0,0,0,0,0,0,0};

void Resolve_ChassisBoard_Data(void)
{
	Lift_Motor_State 					= ChassisBoard_Data[0] & 0x0f;
	Longitudinal_Motor_State 	= (ChassisBoard_Data[0] >> 4) & 0x0f;
	Horizontal_Motor_State    = ChassisBoard_Data[1] & 0x0f;
}