#include "gimbal_task_behaviour.h"
#include "gimbal_task.h"
#include "bsp_math.h"
#include "vision_task.h"

#define yaw_angle gimbal_y.add_angle
#define pitch_angle gimbal_p.add_angle

static void gimbal_control_behaviour_mode(void);
static void gimbal_zero_force_control(float *yaw,float *pitch);
static void gimbal_init_control(float *yaw,float *pitch);
static void IMU_yaw_angle_limit(void);
static void CAN_yaw_angle_limit(void);
static void IMU_pitch_angle_limit(void);
static void CAN_pitch_angle_limit(void);
static int deadline_judge(float a,int flag);
//yaw 1 pitch 2
static int deadline_judge(float a,int flag)
{
	if(flag==2)
	{
		if(abs(a)<=PITCH_DEADLINE) return 0;
	    else return 1;
	}
	else
	{
		if(abs(a)<=YAW_DEADLINE) return 0;
	    else return 1;		
	}

}


void gimbal_control_behaviour(void)
{
	gimbal_control_behaviour_mode();
	/*         ����        */
	if(vision_mode==VISION_OFF)
	{
		if(deadline_judge(yaw_angle,1)!=0)
	    gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
	    if(deadline_judge(pitch_angle,2)!=0)
	    gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle;
	}
	else
	{
		gimbal_y.target_angle = yaw_angle;
		gimbal_p.target_angle = pitch_angle;
	}	
	
	if(gimbal_y.target_angle>180)
		gimbal_y.target_angle -= 360;
	if(gimbal_y.target_angle<-180)
		gimbal_y.target_angle += 360;

}

static void gimbal_control_behaviour_mode(void)
{
	if(gimbal_set_mode==GIMBAL_ZERO_FORCE)
	{
		gimbal_zero_force_control(&yaw_angle,&pitch_angle);
	}
	if(gimbal_set_mode==GIMBAL_INIT)
	{
		gimbal_init_control(&yaw_angle,&pitch_angle);
	}
}


static void gimbal_zero_force_control(float *yaw,float *pitch)
{
	if(yaw == NULL || pitch == NULL)
	{
		return;
	}
	*yaw = 0;
	*pitch = 0;
}
//��Ūpitch��Ūyaw
static void gimbal_init_control(float *yaw,float *pitch)
{
	if(yaw == NULL || pitch == NULL)
	{
		return;
	}
	
	if (fabs(INIT_PITCH_SET - gimbal_p.IMU_actual_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_p.IMU_actual_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_p.IMU_actual_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_y.IMU_actual_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

static void IMU_yaw_angle_limit()
{
	float angle_basic=gimbal_y.target_angle-gimbal_y.IMU_actual_angle;
	if(gimbal_y.IMU_actual_angle+angle_basic+yaw_angle>YAW_ANGLE_MAX)
		if(yaw_angle>0.0f)
		yaw_angle=YAW_ANGLE_MAX-gimbal_y.IMU_actual_angle-angle_basic;
	if(gimbal_y.IMU_actual_angle+angle_basic+yaw_angle<YAW_ANGLE_MIN)
		if(yaw_angle<0.0f)
			yaw_angle=YAW_ANGLE_MIN-gimbal_y.IMU_actual_angle-angle_basic;
}

static void CAN_yaw_angle_limit()
{
	float angle_basic=gimbal_y.target_angle-gimbal_y.CAN_actual_angle;
	if(gimbal_y.IMU_actual_angle+angle_basic+yaw_angle>YAW_ANGLE_MAX)
		if(yaw_angle>0.0f)
		yaw_angle=YAW_ANGLE_MAX-gimbal_y.CAN_actual_angle-angle_basic;
	if(gimbal_y.CAN_actual_angle+angle_basic+yaw_angle<YAW_ANGLE_MIN)
		if(yaw_angle<0.0f)
			yaw_angle=YAW_ANGLE_MIN-gimbal_y.CAN_actual_angle-angle_basic;
}


static void IMU_pitch_angle_limit()
{
	float angle_basic=gimbal_p.target_angle-gimbal_p.IMU_actual_angle;
	if(gimbal_p.IMU_actual_angle+angle_basic+pitch_angle>YAW_ANGLE_MAX)
		if(pitch_angle>0.0f)
		pitch_angle=YAW_ANGLE_MAX-gimbal_p.IMU_actual_angle-angle_basic;
	if(gimbal_p.IMU_actual_angle+angle_basic+pitch_angle<YAW_ANGLE_MIN)
		if(pitch_angle<0.0f)
			pitch_angle=YAW_ANGLE_MIN-gimbal_p.IMU_actual_angle-angle_basic;
}

static void CAN_pitch_angle_limit()
{
	float angle_basic=gimbal_p.target_angle-gimbal_p.CAN_actual_angle;
	if(gimbal_p.CAN_actual_angle+angle_basic+pitch_angle>YAW_ANGLE_MAX)
		if(pitch_angle>0.0f)
		pitch_angle=YAW_ANGLE_MAX-gimbal_p.CAN_actual_angle-angle_basic;
	if(gimbal_p.CAN_actual_angle+angle_basic+pitch_angle<YAW_ANGLE_MIN)
		if(pitch_angle<0.0f)
			pitch_angle=YAW_ANGLE_MIN-gimbal_p.CAN_actual_angle-angle_basic;
}



