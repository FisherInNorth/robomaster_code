#include "Motor_Control.h"

int calibrate_flag = 1;
int calibrate_lift_flag = 1;
int calibrate_chuck_flag = 1;
int calibrate_longitudinal_flag = 1;
uint8_t end_flag = 0;
uint16_t time_mineral_count = 0;

void Motor_Control(void)
{
	if(Lift_Motor_State==3)				//上升
	{
		inside_lift_motorL.apid.target_angle +=80;
		inside_lift_motorR.apid.target_angle -=80;
	}
	else if(Lift_Motor_State==4)	//下降
	{
		inside_lift_motorL.apid.target_angle -=80;
		inside_lift_motorR.apid.target_angle +=80;
	}
	else if(Lift_Motor_State==6)
	{
		inside_lift_motorL.apid.target_angle -=40;
		inside_lift_motorR.apid.target_angle +=40;
	}
	else if(Lift_Motor_State==7)
	{
		inside_lift_motorL.apid.target_angle +=40;
		inside_lift_motorR.apid.target_angle -=40;
	}
	else if(Lift_Motor_State == 5)
	{
		if(Longitudinal_Ready_State == 0)
		{
			if(end_flag == 0)
			{	
			  chuck_roll.apid.target_angle=0;
			  chuck_pitch.apid.target_angle -=60;				
			  inside_lift_motorL.apid.target_angle +=2;
			  inside_lift_motorR.apid.target_angle -=2;
				if((inside_lift_motorL.apid.target_angle > 2000) || (inside_lift_motorR.apid.target_angle < -2000))		
				{
					  inside_lift_motorL.apid.target_angle = 2000;
					  inside_lift_motorR.apid.target_angle = -2000;
						longitudinal_motorL.apid.target_angle +=400;
						longitudinal_motorR.apid.target_angle -= 400;
						if((longitudinal_motorL.apid.target_angle > 700000) || (longitudinal_motorR.apid.target_angle < -700000))
						{
							longitudinal_motorL.apid.target_angle = 700000;
							longitudinal_motorR.apid.target_angle = -700000;
							time_mineral_count++;
							if(time_mineral_count == 2000)
							{
								time_mineral_count = 0;
								end_flag = 1;
							}
						}
					}
				}
				else
				{
			    inside_lift_motorL.apid.target_angle +=30;
			    inside_lift_motorR.apid.target_angle -=30;
					if((inside_lift_motorL.apid.target_angle > 180000) || (inside_lift_motorR.apid.target_angle < -180000))
					{
						inside_lift_motorL.apid.target_angle = 180000;
						inside_lift_motorR.apid.target_angle = -180000;
						chuck_pitch.apid.target_angle +=60;
						if(chuck_pitch.apid.target_angle > 112050)
						{
							chuck_pitch.apid.target_angle = 112050;
							
							longitudinal_motorL.apid.target_angle -=400;
							longitudinal_motorR.apid.target_angle += 400;		
							if((longitudinal_motorL.apid.target_angle < 0) || (longitudinal_motorR.apid.target_angle > 0))
							{
								longitudinal_motorL.apid.target_angle = 0;
								longitudinal_motorR.apid.target_angle = 0;
								Longitudinal_Ready_State = 1;
							}
					 }
					}
				}
			}
		else
		{
			inside_lift_motorL.apid.target_angle -= 80;
			inside_lift_motorR.apid.target_angle += 80;
	  }
	}
	else if(Lift_Motor_State == 0)
	{
		end_flag = 0;
		Longitudinal_Ready_State = 0;
	}
	
	//抬升下限
	if(inside_lift_motorL.apid.target_angle < 0)
	{
		inside_lift_motorL.apid.target_angle = 0;
	}
	if(inside_lift_motorR.apid.target_angle > 0)
	{
		inside_lift_motorR.apid.target_angle = 0;
	}
	//抬升上限
	if(inside_lift_motorL.apid.target_angle > 200000)
	{
		inside_lift_motorL.apid.target_angle = 200000;
	}
	if(inside_lift_motorR.apid.target_angle < -200000)
	{
		inside_lift_motorR.apid.target_angle = -200000;
	}
	
	if(Longitudinal_Motor_State==3)
	{
		longitudinal_motorL.apid.target_angle +=400;
		longitudinal_motorR.apid.target_angle -=400;
	}
	else if(Longitudinal_Motor_State==4)
	{
		longitudinal_motorL.apid.target_angle -=400;
		longitudinal_motorR.apid.target_angle +=400;
	}
	//伸缩电机限位
	if(longitudinal_motorL.apid.target_angle < 0)
	{
		longitudinal_motorL.apid.target_angle = 0;
	}
	if(longitudinal_motorR.apid.target_angle > 0 )
	{
		longitudinal_motorR.apid.target_angle = 0;
	}

	if(longitudinal_motorL.apid.target_angle > 1060000)
		longitudinal_motorL.apid.target_angle = 1060000;
	if(longitudinal_motorR.apid.target_angle < -1060000)
		longitudinal_motorR.apid.target_angle = -1060000;
	if(Horizontal_Motor_State==3)
	{
		horizontal_motor.apid.target_angle +=100;
	}
	else if(Horizontal_Motor_State==4)
	{
		horizontal_motor.apid.target_angle -=100;
	}


	if(Chuck_Yaw_State==3)
	{
		chuck_yaw.apid.target_angle += 100;
	}
	else if(Chuck_Yaw_State==4)
	{
		chuck_yaw.apid.target_angle -= 100;
	}



	if(Chuck_Pitch_State==3)
	{
		chuck_pitch.apid.target_angle += 60;
	}
	else if(Chuck_Pitch_State==4)
	{
		chuck_pitch.apid.target_angle -= 60;
	}
	else if(Chuck_Pitch_State==5)
	{
		chuck_pitch.apid.target_angle = 25000;
	}
//	if(chuck_pitch.apid.target_angle < 0)
//	{
//		chuck_pitch.apid.target_angle = 0;
//	}
	
	if(chuck_pitch.apid.target_angle < 2000)
	{
		chuck_pitch.apid.target_angle = 2000;
	}
	if(chuck_pitch.apid.target_angle > 235000)
	{
		chuck_pitch.apid.target_angle = 235000;
	}

	if(Chuck_Roll_State==3)
	{
		chuck_roll.apid.target_angle += 30;
	}
	else if(Chuck_Roll_State==4)
	{
		chuck_roll.apid.target_angle -= 30;
	}
	else if(Chuck_Roll_State==5)
		chuck_roll.apid.target_angle=0;
	
	if(chuck_roll.apid.target_angle==0)
		Roll_Ready_State=1;
	else
		Roll_Ready_State=0;
	
	if(chuck_roll.apid.target_angle > 77824)
	{
		chuck_roll.apid.target_angle = 77824;
	}
	else if(chuck_roll.apid.target_angle < -77824)
	{
		chuck_roll.apid.target_angle = -77824;
	}

//	if((longitudinal_motorL.apid.total_angle < 1000) || (longitudinal_motorR.apid.total_angle > -1000))
//	{
//		Longitudinal_Ready_State = 1;
//	}
//	else
//	{
//		Longitudinal_Ready_State = 0;
//	}
	
//	if((inside_lift_motorL.apid.total_angle > 170000) || (inside_lift_motorR.apid.total_angle < -170000))
//	{
//		Lift_Ready_State = 1;
//	}
//	else
//	{
//		Lift_Ready_State = 0;
//	}
//	if(PumpAll_State==3)
//	{
//		Pump1_ON;
//		Pump2_ON;
//	}
//	else if(PumpAll_State==0)
//	{
//		Pump1_OFF;
//		Pump2_OFF;
//	}
}

void Motor_Calibrate(void)
{
	inside_lift_motorL.apid.target_angle -=40;
	inside_lift_motorR.apid.target_angle +=40;
  if((abs(inside_lift_motorL.apid.target_angle - inside_lift_motorL.apid.total_angle) > 80000) && (abs(inside_lift_motorR.apid.target_angle - inside_lift_motorR.apid.total_angle) > 80000) )
	{
		if((abs(inside_lift_motorL.vpid.actual_speed) < 200) && (abs(inside_lift_motorR.vpid.actual_speed)) < 200)
		{
			inside_lift_motorL.apid.target_angle = 0;
			inside_lift_motorR.apid.target_angle = 0;
			inside_lift_motorL.apid.actual_angle = 0;
			inside_lift_motorR.apid.actual_angle = 0;
			calibrate_flag = 0;
		}
	}
}

void Motor_Lift_Calibrate(void)
{
	inside_lift_motorL.apid.target_angle -=40;
	inside_lift_motorR.apid.target_angle +=40;
  if((abs(inside_lift_motorL.apid.target_angle - inside_lift_motorL.apid.total_angle) > 20000) && (abs(inside_lift_motorR.apid.target_angle - inside_lift_motorR.apid.total_angle) > 20000))
	{
		if( (abs(inside_lift_motorL.vpid.actual_speed) < 200) && (abs(inside_lift_motorR.vpid.actual_speed) < 200) )
		{
			inside_lift_motorL.apid.target_angle = inside_lift_motorL.apid.total_angle;
			inside_lift_motorR.apid.target_angle = inside_lift_motorR.apid.total_angle;
			inside_lift_motorL.round_cnt = 0;
			inside_lift_motorR.round_cnt = 0;
			calibrate_lift_flag = 0;
		}
	}
}

void Motor_Chuck_Calibrate(void)
{
	chuck_pitch.apid.target_angle -= 40;
	if(((abs(chuck_pitch.apid.target_angle - chuck_pitch.apid.total_angle)) > 40000))
	{
		if(abs(chuck_pitch.vpid.actual_speed) < 200)
		{
			chuck_pitch.apid.target_angle = chuck_pitch.apid.total_angle;
			chuck_pitch.round_cnt = 0;
			calibrate_chuck_flag = 0;
		}
	}
}

void Motor_Longitudinal_Calibrate(void)
{
	longitudinal_motorL.apid.target_angle -=40;
	longitudinal_motorR.apid.target_angle +=40;
	if((abs(longitudinal_motorR.apid.target_angle - longitudinal_motorR.apid.total_angle) > 60000))
	{
		if((abs(longitudinal_motorR.vpid.actual_speed) < 200))
		{
			longitudinal_motorL.apid.target_angle = longitudinal_motorL.apid.total_angle;
			longitudinal_motorR.apid.target_angle = longitudinal_motorR.apid.total_angle;
			longitudinal_motorL.round_cnt = 0;
			longitudinal_motorR.round_cnt = 0;
			calibrate_longitudinal_flag = 0;
		}
	}
	
}
