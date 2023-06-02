#ifndef __UPPERBOARD_H__
#define __UPPERBOARD_H__

#include "DR16.h"

extern unsigned char Lift_Motor_State;						//抬升电机状态
extern unsigned char Longitudinal_Motor_State;		//伸缩电机状态
extern unsigned char Horizontal_Motor_State;			//横移电机状态
extern unsigned char Chuck_Yaw_State;		        //Yaw电机状态
extern unsigned char Chuck_Pitch_State;					//Pitch抬升电机状态
extern unsigned char Chuck_Roll_State;		        //roll电机状态
extern unsigned char PumpAll_State;		        //气泵状态
extern unsigned char View_Servo_State;          //图传舵机,1表示抬升,2表示下降
extern unsigned char Longitudinal_Ready_State;	//伸缩电机一键取矿收回时是否已经完全收回,0表示未完全收回,1表示已完全收回
extern unsigned char Lift_Ready_State;          //抬升电机是否达到预定高度
extern unsigned char Pump_State;                //气泵
extern unsigned char ROLL_Ready_State;          //ROLL可使用	
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
