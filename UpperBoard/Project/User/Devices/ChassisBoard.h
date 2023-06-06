#ifndef __CHASSISBOARD_H__
#define __CHASSISBOARD_H__

extern unsigned char Lift_Motor_State;						//抬升电机状态
extern unsigned char Longitudinal_Motor_State;		//伸缩电机状态
extern unsigned char Horizontal_Motor_State;			//横移电机状态
extern unsigned char Chuck_Yaw_State;		        //Yaw电机状态
extern unsigned char Chuck_Pitch_State;					//Pitch抬升电机状态
extern unsigned char Chuck_Roll_State;		        //roll电机状态
extern unsigned char PumpAll_State;		        //气泵状态
extern unsigned char View_Servo_State;		        //图传舵机
extern unsigned char Longitudinal_Ready_State;	//伸缩电机一键取矿收回时是否已经完全收回
extern unsigned char Lift_Ready_State;          //抬升电机是否达到预定高度
extern unsigned char ChassisBoard_Data[];
extern unsigned char Transmit_ChassisBoard_Data[];
extern unsigned char pChassisBoard_DataH;
extern unsigned char pChassisBoard_DataL;
extern unsigned char Roll_Ready_State;
extern unsigned char Longitudinal_Location;
extern unsigned char Lift_Location;
extern unsigned char Chuck_Roll_Location;
extern unsigned char HP_Zero;
extern unsigned char Pump_State;
extern unsigned char Out_Lift_Motor_State;
extern unsigned char Chassis_vx_State;
extern unsigned char Chassis_vy_State;
extern unsigned char Chassis_wz_State;


void Resolve_ChassisBoard_Data(void);
void Load_ChassisBoard_Data(void);

#endif
