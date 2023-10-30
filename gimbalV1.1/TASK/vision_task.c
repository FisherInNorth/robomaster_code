#include "vision_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "bsp_math.h"
#include "chassis_task.h"
#include <math.h>
VISION_t vision_mode=VISION_OFF;
VISION_GET_t vision_sent;

float chassis_angle=0;//底盘速度相对于云台参考系的夹角
float vision_x=0,vision_y=0;//底盘速度在云台坐标系下的速度
int vision_cnt=0;
int vision_delay(int vt)
{
	vision_cnt++;
	if(vision_cnt>vt)  
	{
		vision_cnt=0;
	    return 1;
	}
	else return 0;
}
int yu=0;
void Vision_Task(void)
{
	if(vision_mode==VISION_OFF)
	{
		gimbal_y.add_angle=rc_sent.yaw.target_angle;
		
		gimbal_p.add_angle=rc_sent.pitch.target_angle;
	//	gimbal_p.target_speed=yu;
//		gimbal_y.target_speed=rc_sent.yaw.target_speed;
	}
	else
	{
		//增量
/*		if(vision_delay(40)==1) //20ms
		{
			if(vision_sent.pitch.target_angle>2.0f) 
				vision_sent.pitch.target_angle=vision_sent.pitch.target_angle*0.1+2.0f;
			if(vision_sent.yaw.target_angle>2.0f) 
				vision_sent.yaw.target_angle=vision_sent.yaw.target_angle*0.1+2.0f;
			
		}
        else
		{
			gimbal_y.add_angle=vision_sent.yaw.target_angle*0.2f;
		
		    gimbal_p.add_angle=vision_sent.pitch.target_angle*0.2f;
		}
*/
		//位置
		gimbal_y.add_angle=vision_sent.yaw.target_angle;
		gimbal_p.add_angle=vision_sent.pitch.target_angle;
		//vision_sent.yaw.target_angle=0;
		//vision_sent.pitch.target_angle=0;
	}
}
float vision_actual_angle_cal()
{
	chassis_angle=gimbal_y.CAN_actual_angle-343;
	if(chassis_angle<0)
		chassis_angle+=360;
	return 0;
}

double vision_actual_speed,vision_actual_angle;
void vision_getSpeed()
{
	vision_actual_speed=(double)(chassis_speed_x*chassis_speed_x+chassis_speed_y*chassis_speed_y);
	vision_actual_speed=sqrt(vision_actual_speed);
	vision_x=chassis_speed_x*acos(chassis_angle)-chassis_speed_y*asin(chassis_angle);
	vision_y=chassis_speed_x*asin(chassis_angle)+chassis_speed_y*acos(chassis_angle);
	vision_actual_angle_cal();
	vision_actual_angle=chassis_angle;
}

