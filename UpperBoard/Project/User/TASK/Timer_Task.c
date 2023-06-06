#include "Timer_Task.h"
#include "bsp_servo.h"
#include <math.h>
int time_count = -1;

int target;
int actual;
int error;
int out;
unsigned char Last_HP_Zero=0;

extern RC_ctrl_t rc_ctrl;
extern float chuck_pitch_vPID_Parameters[6];
extern float Longitudinal_MotorL_aPID_Parameters[6];
extern float Longitudinal_MotorR_aPID_Parameters[6];

void Timer_Task()
{
	if(Last_HP_Zero == 1)
	{
		if(HP_Zero == 0)
		{
			calibrate_chuck_flag = 1;
			calibrate_lift_flag = 1;
			calibrate_longitudinal_flag = 1;
			start_flag = 0;
			time_count = -1;
		}
	}
	Last_HP_Zero = HP_Zero;
	if((calibrate_lift_flag == 0) && (calibrate_chuck_flag == 0) && (calibrate_longitudinal_flag == 0))
	{
	//初始化
		if(time_count == -1)
		{
			inside_lift_motorL.round_cnt = -2;
			inside_lift_motorR.round_cnt = 2;
			longitudinal_motorL.round_cnt = -5;
			longitudinal_motorR.round_cnt = 5;

//			chuck_pitch.round_cnt = -2;
			chuck_pitch_vPID_Parameters[5] = 9000;
			Longitudinal_MotorL_aPID_Parameters[5] = 2500;
			Longitudinal_MotorR_aPID_Parameters[5] = 2500;

		}
	
		time_count++;
		
		Longitudinal_Location = abs(longitudinal_motorL.apid.total_angle)/4141;
		Lift_Location = abs(inside_lift_motorL.apid.total_angle)/782;
		Chuck_Roll_Location = abs(chuck_roll.apid.total_angle+77824)/609;
	
		if(time_count%7 == 0)
		{
			GetDR16_Data();
//			CAN1_Motor_ControlMsg();
//			CAN2_Motor_ControlMsg();
		}
	
		if(time_count%30 == 0)
		{
			RemoteChassisBoard();
			Resolve_ChassisBoard_Data();
			//设置舵机角度
			
		}
			Motor_Control();
//	if(time_count%33 == 0)
//	{
//		referee_unpack_fifo_data();
//		Custom_Robot_Control();
//	}
//	
	
	
	if(time_count%50== 0)
	{
		//LED_Green_Toggle();
		SERVO_SET_ANGLE();
	}
//	
//	
//	if(Lift_Motor_State == 3)
//	{
//		LED_Red(1);
//		LED_Blue(0);
//	}
//	else if(Lift_Motor_State == 4)
//	{
//		LED_Red(0);
//		LED_Blue(1);
//	}
//	else
//	{
//		LED_Red(0);
//		LED_Blue(0);
//	}
	
		if(time_count>=1000)
		{
			time_count = 1;
		}
	}
	else
	{
		if(start_flag==1)
		{
			if(calibrate_lift_flag == 1)
			{
				Motor_Lift_Calibrate();
			}
			if(calibrate_chuck_flag == 1)
			{
				chuck_pitch_vPID_Parameters[5] = 2000;
				Motor_Chuck_Calibrate();
			}
			if(calibrate_longitudinal_flag == 1)
			{
				Motor_Longitudinal_Calibrate();
			}
			CAN1_Motor_ControlMsg();
			CAN2_Motor_ControlMsg();
		}
	}
	CAN1_Motor_ControlMsg();
	if(start_flag==1)
		CAN2_Motor_ControlMsg();
}
