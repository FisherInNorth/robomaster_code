#include "ChassisMove_Task.h"


float chassis_vx,chassis_vy,chassis_wz;
float real_chassis_vx,real_chassis_vy,real_chassis_wz;

float max_d_speed_x;
float max_d_speed_y;
float max_d_speed_x_stop;
float max_d_speed_y_stop;
float max_d_speed_z = 10.0f;


static void speed_optimize(void);
static void speed_z_optimize(void);
static void judge_height(void);


   //*****高度判断*****//
static void judge_height(void)
{
	if(outboard_lift_motorL.round_cnt<-12)
	{
	  max_d_speed_x=1.0f;
		max_d_speed_y=2.0f;
		max_d_speed_x_stop=4.0f;
		max_d_speed_y_stop=5.0f;
	}
	else
	{
	  max_d_speed_x=1.0f;
		max_d_speed_y=2.0f;
		max_d_speed_x_stop=5.0f;
		max_d_speed_y_stop=6.0f;
	}
}



  //*****速度优化函数*****//
static void speed_optimize(void)
{
	judge_height();
	
	static float last_xspeed,last_yspeed;
	
	if((chassis_vx!=0)||(chassis_vy!=0))
	{
		if(abs(chassis_vx-last_xspeed)>max_d_speed_x)
		{
			if(chassis_vx>last_xspeed)
				real_chassis_vx+=max_d_speed_x;
			else if(chassis_vx<-last_xspeed)
				real_chassis_vx-=max_d_speed_x;
		}
		else real_chassis_vx=chassis_vx;

		if(abs(chassis_vy-last_yspeed)>max_d_speed_y)
		{
			if(chassis_vy>last_yspeed)
				real_chassis_vy+=max_d_speed_y;
			else if(chassis_vy<-last_yspeed)
				real_chassis_vy-=max_d_speed_y;
		}
		else real_chassis_vy=chassis_vy;
		
		last_xspeed=real_chassis_vx;
		last_yspeed=real_chassis_vy;
	}
	else
	{

		if(real_chassis_vx>0)
		{
			real_chassis_vx-=max_d_speed_x_stop;
			if(real_chassis_vx<0)
				real_chassis_vx=0;
		}
		else 
		{
			real_chassis_vx+=max_d_speed_x_stop;
			if(real_chassis_vx>0)
				real_chassis_vx=0;
		}
		
		if(real_chassis_vy>0)
		{
			real_chassis_vy-=max_d_speed_y_stop;
			if(real_chassis_vy<0)
				real_chassis_vy=0;
		}
		else 
		{
			real_chassis_vy+=max_d_speed_y_stop;
			if(real_chassis_vy>0)
				real_chassis_vy=0;
		}
	}

	chassis_vx=real_chassis_vx;
	chassis_vy=real_chassis_vy;
}

static void speed_z_optimize(void)
{
	static float last_zspeed;
		if(abs(chassis_wz-last_zspeed)>max_d_speed_z)
		{
			if(chassis_wz>last_zspeed)
				real_chassis_wz+=max_d_speed_z;
			else if(chassis_wz<-last_zspeed)
				real_chassis_wz-=max_d_speed_z;
		}
		else real_chassis_wz=chassis_wz;	
		last_zspeed=real_chassis_wz;
		chassis_wz=real_chassis_wz;
}

void ChassisMove(void)
{
	
	speed_optimize();
//	speed_z_optimize();
	
	//int max;
		//速度换算，运动学分解
	BaseVel_To_WheelVel(chassis_vx, chassis_vy, chassis_wz);
	
	//max=find_max();
//	if(max>MAX_MOTOR_SPEED)
//	{
//		chassis_motor1.target_speed=(int)(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
//		chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
//		chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
//		chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
//	}
//	set_chassis_speed(chassis_motor1.target_speed, chassis_motor2.target_speed, chassis_motor3.target_speed, chassis_motor4.target_speed);
}
