#include "Timer_Task.h"
#include "bsp_servo.h"
#include "referee.h"
#include "pid.h"
#include "DR16.h"
int time_count = -1;
int refree_time_count = 0;
unsigned char Last_HP_Zero=0;

int target;
int actual;
int error;
int out;

extern float Outboard_MotorL_aPID_Parameters[];
extern float Outboard_MotorR_aPID_Parameters[];

uint8_t servo_angle;
extern RC_ctrl_t rc_ctrl;
extern int Key_Mode;

void Timer_Task()
{
	
//	if(time_count%7==0)
//	{
//		judge_v();
//	}
//	
	
	
	
	refree_time_count++;
	if(robot_state.remain_HP == 0)
	{
		HP_Zero = 1;
	}
	else
	{
		HP_Zero = 0;
	}
	
		if(Last_HP_Zero == 1)
	{
		if(HP_Zero == 0)
		{
		calibrate_lift_flag = 1;
		start_flag = 0;
		time_count = -1;
		}
	}
	Last_HP_Zero = HP_Zero;

	if(refree_time_count % 15 == 0)
	{
		referee_unpack_fifo_data();
	}
	if(refree_time_count >= 1000)
	{
		refree_time_count = 1;
	}
	
	if(calibrate_lift_flag == 0)
	{
		//初始化
		if(time_count == -1)
		{
//			outboard_lift_motorL.round_cnt = 1;
//			outboard_lift_motorR.round_cnt = -1;
		}
		
		time_count++;
		
		if(time_count%15 == 0)
		{
//			if(Key_Mode == 0)
//			{
//			HandleDR16_Data();			
//			}
//			else
//			{
//			Key_Control();			
//			}
//			Key_Mode_Judge();
			ChassisMove();
			Motor_Control();
		}
		
		//外部抬升电机
		if(time_count%1 == 0)
		{
//			if(rc_ctrl.rc.s[1] == 1)
//			{
//				if(rc_ctrl.rc.s[0] == 1)
//				{
//				outboard_lift_motorL.apid.target_angle -=150;
//				outboard_lift_motorR.apid.target_angle +=150;
//				}

//			  if(rc_ctrl.rc.s[0] == 2)
//			  {
//				outboard_lift_motorL.apid.target_angle +=150;
//				outboard_lift_motorR.apid.target_angle -=150;
//				}
			
//		  }
			//抬升下限
			if(outboard_lift_motorL.apid.target_angle>-12192)
			{
				outboard_lift_motorL.apid.target_angle = -12192;
			}
			if(outboard_lift_motorR.apid.target_angle<12192)
			{
				outboard_lift_motorR.apid.target_angle = 12192;
			}
			
			//抬升上限
			if(outboard_lift_motorL.apid.target_angle<-300000)
			{
				outboard_lift_motorL.apid.target_angle = -300000;
			}
			if(outboard_lift_motorR.apid.target_angle>300000)
			{
				outboard_lift_motorR.apid.target_angle = 300000;
			}
		}
		
		if(time_count%25 == 0)
		{
			RemoteUpperBoard();
			Resolve_UpperBoard_Data();
		}
		
		
		if(time_count == 500)
		{
			LED_Green_Toggle();
			
		}
		

		
		if(time_count>=1000)
		{
			time_count = 1;
		}
		abs_motor();
	}
	else
	{
		if(start_flag == 2)
			Motor_Lift_Calibrate();
	}
	if(start_flag==2)
		CAN2_Motor_ControlMsg();
	CAN1_Motor_ControlMsg();
}
