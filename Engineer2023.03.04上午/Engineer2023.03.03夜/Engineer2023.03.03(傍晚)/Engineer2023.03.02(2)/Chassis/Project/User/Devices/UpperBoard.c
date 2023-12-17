#include "UpperBoard.h"

/*
模式一：左拨杆开关为中时
1.左摇杆控制前后左右运动
2.右摇杆控制左右转向、

模式二：左拨杆开关为上时
1.左摇杆控制大臂上下、前后运动
2.右摇杆控制小臂

*/

/*
0:静止
1:正向微调
2:反向微调
3:正向快速运动
4:反向快速运动
*/
unsigned char Lift_Motor_State;						//抬升电机状态
unsigned char Longitudinal_Motor_State;		//伸缩电机状态
unsigned char Horizontal_Motor_State;     //横向伸缩电机状态

unsigned char UpperBoard_Data[8] = {0,0,0,0,0,0,0,0};

void Load_UpperBoard_Data(void)
{
	UpperBoard_Data[0] = 0;
	UpperBoard_Data[0] |= Lift_Motor_State;
	UpperBoard_Data[0] |= Longitudinal_Motor_State<<4;
	UpperBoard_Data[1] = 0;
	UpperBoard_Data[1] |= Horizontal_Motor_State;
}

