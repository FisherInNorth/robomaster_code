#include "UpperBoard.h"

/*
0:静止
1:正向微调
2:反向微调
3:正向快速运动
4:反向快速运动
...(最多支持16种状态)...
*/

/*
发送格式：
每个字节是独立的数据帧
前4位是操作ID，即描述的是哪个操作，最多可控制16种操作
后4位是状态码，即该操作处于什么状态，最多可描述16种状态
*/

unsigned char Lift_Motor_State=0;						//抬升电机状态
unsigned char Longitudinal_Motor_State=0;		//伸缩电机状态
unsigned char Horizontal_Motor_State=0;			//横移电机状态
unsigned char Chuck_Yaw_State=0;		        //Yaw电机状态
unsigned char Chuck_Pitch_State=0;					//Pitch抬升电机状态
unsigned char Chuck_Roll_State=0;		        //roll电机状态
unsigned char PumpAll_State=0;		        //气泵状态
unsigned char View_Servo_State=0;          //图传舵机,1表示抬升,2表示下降
unsigned char Longitudinal_Ready_State=0;	//伸缩电机一键取矿收回时是否已经完全收回,0表示未完全收回,1表示已完全收回
unsigned char Lift_Ready_State = 0;				//抬升电机是否抬升到指定位置
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

			//...(最多支持十六个)...
			default:break;
		}
	}
}