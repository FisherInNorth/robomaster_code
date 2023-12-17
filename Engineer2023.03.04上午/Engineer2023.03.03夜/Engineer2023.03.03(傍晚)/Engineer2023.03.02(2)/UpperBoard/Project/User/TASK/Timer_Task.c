#include "Timer_Task.h"

unsigned int time_count = 0;

int target;
int actual;
int error;
int out;

extern RC_ctrl_t rc_ctrl;

float Ren=0;//²âÊÔÓÃÀ´»ØÒÆ¶¯²ÎÊý
int Ren_flag=0;
void Timer_Task()
{

	time_count++;
	
	
	if(time_count%20 == 0)
	{
		GetDR16_Data();
		//ChassisMove();
	}
	
	if(time_count%20 == 0)
	{
		if(rc_ctrl.rc.s[0] == 1)
		{
			inside_lift_motorL.apid.target_angle +=20;
			inside_lift_motorR.apid.target_angle -=20;
		}
		else if(rc_ctrl.rc.s[0] == 2)
		{
			inside_lift_motorL.apid.target_angle -=20;
			inside_lift_motorR.apid.target_angle +=20;
		}
		else
		{
			inside_lift_motorL.apid.target_angle = inside_lift_motorL.apid.total_angle;
			inside_lift_motorR.apid.target_angle = inside_lift_motorR.apid.total_angle;
		}
		if(rc_ctrl.rc.s[1] == 1)
		{
			horizontal_motor.apid.target_angle +=300;
		}
		else if(rc_ctrl.rc.s[1] == 2)
		{

			horizontal_motor.apid.target_angle -=300;
		}
		else
		{
			horizontal_motor.apid.target_angle = horizontal_motor.apid.total_angle;
		}
		
		
			/****×óÒ¡¸Ë×óÓÒ¿ØÖÆyaw****/
		if((rc_ctrl.rc.ch[2] > 1374))
		{
			chuck_yaw.apid.target_angle += 50;
		}
		else if(rc_ctrl.rc.ch[2] < 674)
		{
			chuck_yaw.apid.target_angle -= 50;
		}	
		else
		{
			chuck_yaw.apid.target_angle = chuck_yaw.apid.total_angle;
		}

			/****×óÒ¡¸Ë×óÓÒ¿ØÖÆyaw****/
		if((rc_ctrl.rc.ch[3] > 1374))
		{
			chuck_pitch.apid.target_angle += 5;
		}
		else if(rc_ctrl.rc.ch[3] < 674)
		{
			chuck_pitch.apid.target_angle -= 5;
		}	
		else
		{
			chuck_pitch.apid.target_angle = chuck_pitch.apid.total_angle;
		}
		
			/****×óÒ¡¸Ë×óÓÒ¿ØÖÆyaw****/
		if((rc_ctrl.rc.ch[0] > 1374))
		{
			chuck_roll.vpid.target_speed = 20;
		}
		else if(rc_ctrl.rc.ch[0] < 674)
		{
			chuck_roll.vpid.target_speed = 20;
		}	
		else
		{
			chuck_roll.vpid.target_speed = 0;
		}
		
	}
	if(time_count == 500)
	{
		LED_Green_Toggle();
		
//		chassis_motor1.integral_max = 10000;
//		chassis_motor1.vPID_max = 10000;
////		chassis_motor1.vpid.target_speed = 300;
//		
//		chassis_motor2.integral_max = 10000;
//		chassis_motor2.vPID_max = 10000;
////		chassis_motor2.vpid.target_speed = 300;
//		
//		chassis_motor3.integral_max = 10000;
//		chassis_motor3.vPID_max = 10000;
////		chassis_motor3.vpid.target_speed = 300;
//		
//		chassis_motor4.integral_max = 10000;
//		chassis_motor4.vPID_max = 10000;
////		chassis_motor4.vpid.target_speed = 300;
//		outboard_lift_motorL.apid.target_angle = -8000;
//		outboard_lift_motorR.apid.target_angle = 8000;
//		target = outboard_lift_motorL.apid.target_angle;
//		actual = outboard_lift_motorL.apid.total_angle;
//		error = outboard_lift_motorL.apid.err;
//		out = outboard_lift_motorL.apid.PID_OUT;
	}
	
	
	
	
	if(time_count>=1000)
	{
		time_count = 1;
		++Ren_flag;
	}
	
	if(Ren_flag%2==0)
	{
		Ren=100;
		if(Ren_flag%4==0)Ren=-100;
	}
	
}
