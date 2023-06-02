#include "Timer_Task.h"
#include "bsp_servo.h"
#include "referee.h"
int time_count = -1;

int target;
int actual;
int error;
int out;

uint8_t servo_angle;
extern RC_ctrl_t rc_ctrl;
extern int Key_Mode;
void Timer_Task()
{


	//初始化
	if(time_count == -1)
	{
		outboard_lift_motorL.round_cnt = 1;
		outboard_lift_motorR.round_cnt = -1;
	}
	
	time_count++;
	
	if(time_count%15 == 0)
	{
		if(Key_Mode == 0)
		{
		HandleDR16_Data();			
		}
		else
		{
		Key_Control();			
		}
		Key_Mode_Judge();
		referee_unpack_fifo_data();
		ChassisMove();
	}
	
	//外部抬升电机
	if(time_count%1 == 0)
	{
		if(rc_ctrl.rc.s[1] == 1)
		{
			if(rc_ctrl.rc.s[0] == 1)
			{
			outboard_lift_motorL.apid.target_angle -=150;
			outboard_lift_motorR.apid.target_angle +=150;
			}

		  if(rc_ctrl.rc.s[0] == 2)
		  {
			outboard_lift_motorL.apid.target_angle +=150;
			outboard_lift_motorR.apid.target_angle -=150;
			}
		
	  }
		//抬升下限
		if(outboard_lift_motorL.apid.target_angle>-4000)
		{
			outboard_lift_motorL.apid.target_angle = -4000;
		}
		if(outboard_lift_motorR.apid.target_angle<4000)
		{
			outboard_lift_motorR.apid.target_angle = 4000;
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
