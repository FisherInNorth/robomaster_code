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
extern unsigned char Longitudinal_Location;			//伸出位置
extern unsigned char Lift_Location;							//内部抬升位置
extern unsigned char Chuck_Roll_Location;				//机械臂roll轴位置
extern unsigned char HP_Zero;										//零血量标志位
extern unsigned char Out_Lift_Motor_State;			//外抬升电机状态
extern unsigned char Chassis_vx_State;					//底盘x轴运动状态
extern unsigned char Chassis_vy_State;					//底盘y轴运动状态
extern unsigned char Chassis_wz_State;					//底盘z轴运动状态
extern unsigned char Longitudinal_Location_Low;	//伸缩位置低8位
extern unsigned char Longitudinal_Location_High;//伸缩位置高8位
extern unsigned char Lift_Location_Low;					//内部抬升位置低8位
extern unsigned char Lift_Location_High;				//内部抬升位置高8位
extern unsigned char Lift_Location_UI;					//抬升总位置
extern unsigned char Chuck_Roll_Location_Low;		//机械臂roll轴位置低8位
extern unsigned char Chuck_Roll_Location_High;	//机械臂roll轴位置高8位
extern uint16_t Chuck_Roll_Location_UI;					//将机械臂roll轴的标志位转为360度数制
extern unsigned char UpperBoard_Data[];					//向云台发送数据数组
extern unsigned char Receive_UpperBoard_Data[];	//云台数据接收数组
extern unsigned char pUpperBoard_DataH;					//云台数据暂时接收数组
extern unsigned char pUpperBoard_DataL;					//云台数据暂时接收数组

void Load_UpperBoard_Data(void);
void Resolve_UpperBoard_Data(void);




#endif
