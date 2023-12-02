#include "Timer_Task.h"

unsigned int time_count = 0;

int target;
int actual;
int error;
int out;

extern RC_ctrl_t rc_ctrl;

void Timer_Task()
{

	time_count++;
	
	if(time_count%7 == 0)
	{
		GetDR16_Data();
		ChassisMove();
	}
	
	if(time_count%1 == 0)
	{
		if(rc_ctrl.rc.s[0] == 1)
		{
			outboard_lift_motorL.apid.target_angle -=20;
			outboard_lift_motorR.apid.target_angle +=20;
		}
		if(rc_ctrl.rc.s[0] == 2)
		{
			outboard_lift_motorL.apid.target_angle +=20;
			outboard_lift_motorR.apid.target_angle -=20;
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
		target = outboard_lift_motorL.apid.target_angle;
		actual = outboard_lift_motorL.apid.total_angle;
		error = outboard_lift_motorL.apid.err;
		out = outboard_lift_motorL.apid.PID_OUT;
	}
	
	
	
	
	if(time_count>=1000)
	{
		time_count = 1;
	}
}
