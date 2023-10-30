#include "shoot_task.h"
#include "remote_control.h"
#include "sent_task.h"
#include "gimbal_task.h"
uint8_t data[2]={1,0};
shoot_task_t rc_shoot;
int shoot_times=0;
int One_Shoot_flag=0;
int Ten_Shoot_flag=0;

float TriggerSpeed[5] 	= {6.0f,			0.08f,			0.0f,		5100,			500};  //�������ٶȻ�
float TriggerAngle[5] 	= {0.18f,			0.0f,			0.0f,		3500,			0};  //�����ֽǶȻ�

float FricLeftSpeed[5] 	= {4.5f,      0.95f,      0.2f,    3000.0f,      500.0f};  //��Ħ�����ٶȻ�
float FricRightSpeed[5] 	= {10.0f,      0.2f,      0.3f,    3000.0f,      500.0f};  //��Ħ�����ٶȻ�
static void trigger_angle_set(void);
//���·��ӵ�����
static void shoot_angle_clc(void);
//�ſ���
static void prevent_jam(void);
//Ħ����PID����
static void fric_pid(void);
//������PID����
static void trigger_pid(void);
void prevent_jam()
{
	if(rc_shoot.trigger.last_back_flag==0)
	{
		if(rc_ctrl.rc.s[1]==2)  
		{
			int angle;
			rc_shoot.trigger.last_back_flag=1;
			angle=((1*36*36)/360*8191);
			if(rc_shoot.trigger.rounds>=40)
			{
				rc_shoot.trigger.target_angle-=rc_shoot.trigger.rounds*8192;
				if(rc_shoot.trigger.target_angle<=0) rc_shoot.trigger.target_angle=1;
				rc_shoot.trigger.total_angle -=rc_shoot.trigger.rounds*8192 ;
					rc_shoot.trigger.begin_angle=rc_shoot.trigger.total_angle;
				rc_shoot.trigger.rounds=0;
			}
			rc_shoot.trigger.target_angle-=angle;
		}
	}
	
	if(rc_ctrl.rc.s[1]!=2) rc_shoot.trigger.last_back_flag=0;
	
}
void shoot_task(void)
{
	shoot_angle_clc();
	fric_pid();
	trigger_pid();
	canTX_fric(rc_shoot.left_fric.set_currunt,rc_shoot.right_fric.set_currunt,rc_shoot.trigger.set_currunt);
	//canTX_gimbal(gimbal_p.given_current,gimbal_y.given_current);
//	prevent_jam();
}
void shoot_init(void)
{
	
	PID_Init(&rc_shoot.trigger.speed_pid,TriggerSpeed);
	PID_Init(&rc_shoot.trigger.angle_pid,TriggerAngle);
	
  PID_Init(&rc_shoot.left_fric.speed_pid,FricLeftSpeed);
	PID_Init(&rc_shoot.right_fric.speed_pid,FricRightSpeed);
	
	rc_shoot.trigger.actual_angle = 0.0f;
	rc_shoot.trigger.actual_speed = 0.0f;
	
	rc_shoot.trigger.last_shoot_flag = 0;
	rc_shoot.trigger.last_back_flag = 0;
	rc_shoot.trigger.target_angle = 0.0f;
	rc_shoot.trigger.target_speed = 0.0f;
	
	rc_shoot.left_fric.actual_speed = 0.0f;
	rc_shoot.left_fric.target_speed = -SHOOT_FRIC_SPEED;
	
	rc_shoot.right_fric.actual_speed = 0.0f;
	rc_shoot.right_fric.target_speed = SHOOT_FRIC_SPEED;

}

static void shoot_angle_clc(void)
{
	if(rc_shoot.trigger.last_shoot_flag==0)
	{
		if(One_Shoot_flag==1||Ten_Shoot_flag==1)  
		{
			rc_shoot.trigger.last_shoot_flag=1;
			trigger_angle_set();
			shoot_times+=1;
			if(shoot_times>6) shoot_times=0;
		}
	}
	
	if(One_Shoot_flag!=1 && Ten_Shoot_flag!=1) rc_shoot.trigger.last_shoot_flag=0;

}

static void fric_pid(void)
{
	PID_Calc(&rc_shoot.left_fric.speed_pid,rc_shoot.left_fric.target_speed,rc_shoot.left_fric.actual_speed);
	
	rc_shoot.left_fric.set_currunt=rc_shoot.left_fric.speed_pid.out;
	
	PID_Calc(&rc_shoot.right_fric.speed_pid,rc_shoot.right_fric.target_speed,rc_shoot.right_fric.actual_speed);
	
	rc_shoot.right_fric.set_currunt=rc_shoot.right_fric.speed_pid.out;
}


static void trigger_pid(void)
{
	PID_Calc(&rc_shoot.trigger.angle_pid,rc_shoot.trigger.target_angle,rc_shoot.trigger.total_angle);
	
	rc_shoot.trigger.target_speed = rc_shoot.trigger.angle_pid.out;
	
	PID_Calc(&rc_shoot.trigger.speed_pid,rc_shoot.trigger.target_speed,rc_shoot.trigger.actual_speed);
	
	rc_shoot.trigger.set_currunt = rc_shoot.trigger.speed_pid.out;
}

float watch;
static void trigger_angle_set(void)
{
	float angle_max;
	angle_max=((SHOOT_NUM*36.0f*36.0f)/360.0f*8192.0f);
	
	watch=angle_max;
	if(rc_shoot.trigger.rounds>=40)
	{
		rc_shoot.trigger.target_angle-=30*8192;
		if(rc_shoot.trigger.target_angle<=0) 
			rc_shoot.trigger.target_angle=1;
		
		rc_shoot.trigger.total_angle -=30*8192;
	    rc_shoot.trigger.begin_angle=rc_shoot.trigger.total_angle;
		rc_shoot.trigger.rounds-=30;
	}
	
	if(One_Shoot_flag==1)
	{
		rc_shoot.trigger.target_angle+=angle_max;
	}
	else if(Ten_Shoot_flag==1)
	{
		rc_shoot.trigger.target_angle+=((SHOOT_NUM*10*36*36)/360*8191);
	}
	
	
	
	
}

void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->rounds ++;
	motor->total_angle = motor->rounds * 8192 + motor->actual_angle;

	
}

