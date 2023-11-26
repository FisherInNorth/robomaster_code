#include "BSP_Flip.h"
#include "BSP_Motor.h"
#include "BSP_Can.h"
#include "BSP_Clamp.h"
#include "BSP_Mineral.h"
#include "Remote_Task.h"

MOTOR_MOVE_t flip_move;
MOTOR_t flip_R_motor;
MOTOR_t flip_L_motor;

uint8_t Flip_PID2_Flag=0;
uint8_t current_msg2[8];
int Flip_Angle_R = 0;
int Flip_Angle_L = 0;

float Flip_Kp_L=40;
float Flip_Kp_R=55;
float Flip_Ki_L=3.7f;
float Flip_Ki_R=1.2f;
float Flip_Kd=22;
float Flip_A_Kp=1;
float Flip_A_Ki=0.5f;
float Flip_A_Kd=0.15f;

float Flip_kp_less=20;
float Flip_ki_less=4.2;
float Flip_kd_less=3.7;
int FLIP_SPEED=0;//测试用
/**
	* @function函数:Flip_Init
	* @brief描述:机械臂初始化`21
	* @param输入:无
	* @retval返回值:无
  */
void Flip_Init(void)
{
	flip_move=stop;
	flip_R_motor.target_current=0;
	flip_R_motor.actual_current=0;
	flip_R_motor.round_cnt=0.0f;
	flip_R_motor.vpid.target_speed=0;
	flip_R_motor.vpid.actual_speed=0.0f;
	flip_R_motor.vpid.err=0;
	flip_R_motor.vpid.last_err=0;
	flip_R_motor.vpid.err_integration=0;
	flip_R_motor.vpid.P_OUT=0;
	flip_R_motor.vpid.I_OUT=0;
	flip_R_motor.vpid.D_OUT=0;
	flip_R_motor.vpid.PID_OUT=0;
	flip_R_motor.vPID_max=6000;
	flip_L_motor.target_current=0;
	flip_L_motor.actual_current=0;
	flip_L_motor.round_cnt=0.0f;
	flip_L_motor.vpid.target_speed=0;
	flip_L_motor.vpid.actual_speed=0.0f;
	flip_L_motor.vpid.err=0;
	flip_L_motor.vpid.last_err=0;
	flip_L_motor.vpid.err_integration=0;
	flip_L_motor.vpid.P_OUT=0;
	flip_L_motor.vpid.I_OUT=0;
	flip_L_motor.vpid.D_OUT=0;
	flip_L_motor.vpid.PID_OUT=0;
	flip_L_motor.vPID_max=6000;
	
	flip_R_motor.aPID_OUT_MAX=23000;
	flip_R_motor.apid.err=0;
	flip_R_motor.apid.last_err=0;
	flip_R_motor.apid.err_integration=0;
	flip_R_motor.apid.actual_angle=0;
	flip_R_motor.apid.target_angle=0;
	flip_R_motor.apid.P_OUT=0;
	flip_R_motor.apid.I_OUT=0;
	flip_R_motor.apid.D_OUT=0;
	flip_R_motor.apid.PID_OUT=0;
  flip_R_motor.apid.total_angle=0;	
	flip_L_motor.aPID_OUT_MAX=23000;
	flip_L_motor.apid.err=0;
	flip_L_motor.apid.last_err=0;
	flip_L_motor.apid.err_integration=0;
	flip_L_motor.apid.actual_angle=0;
	flip_L_motor.apid.target_angle=0;
	flip_L_motor.apid.P_OUT=0;
	flip_L_motor.apid.I_OUT=0;
	flip_L_motor.apid.D_OUT=0;
	flip_L_motor.apid.PID_OUT=0;
  flip_L_motor.apid.total_angle=0;	
}

/**
	* @function函数:Set_Flip_Speed
	* @brief描述:机械臂伸出速度控制
	* @param输入:机械臂速度
	* @retval返回值:无
  */
void Set_Flip_Speed(int target_speed)
{
	flip_L_motor.vpid.target_speed = target_speed*19;	
	flip_R_motor.vpid.target_speed = -target_speed*19; 
}

/**
	* @function函数:Vpid_Flip_Realize
	* @brief描述:机械臂电机速度pid实现
	* @param输入:机械臂电机速度pid系数
	* @retval返回值:无
  */
void Vpid_Flip_Realize(float kp,float ki,float kd)
{
	Vpid_Realize(&flip_R_motor, kp, ki, kd);
	Vpid_Realize(&flip_L_motor, kp, ki, kd);
}

/**
	* @function函数:Set_Flip_Angle
	* @brief描述:机械臂伸出速度控制
	* @param输入:机械臂速度
	* @retval返回值:无
  */

void Set_Flip_Angle_R(int target_angle)
{
	flip_R_motor.apid.target_angle = target_angle; 
}

void Set_Flip_Angle_L(int target_angle)
{
	flip_L_motor.apid.target_angle = -target_angle;
}
	

/**
	* @function函数:Apid_Flip_Realize
	* @brief描述:机械臂电机速度pid实现
	* @param输入:机械臂电机速度pid系数
	* @retval返回值:无
  */
void Apid_Flip_Realize()
{
	Apid_Realize(&flip_L_motor, Flip_A_Kp, Flip_A_Ki, Flip_A_Kd);
	Apid_Realize(&flip_R_motor, Flip_A_Kp, Flip_A_Ki, Flip_A_Kd);
	Set_Flip_Speed(flip_L_motor.apid.PID_OUT);
	Set_Flip_Speed(flip_R_motor.apid.PID_OUT);
// 	Clamp.c里面也写了，重复了	
//	if(Clamp_Judge==1)
//	{
//		Flip_PID2_Flag=1;
//	}
	if(Flip_PID2_Flag==1)
	{
		Vpid_Realize(&flip_L_motor, Flip_Kp_L, Flip_Ki_L, Flip_Kd);
		Vpid_Realize(&flip_R_motor, Flip_Kp_R, Flip_Ki_R, Flip_Kd);
	}
	else if(Flip_PID2_Flag==0)
	{
		Vpid_Realize(&flip_L_motor, Flip_kp_less, Flip_ki_less, Flip_kd_less);
		Vpid_Realize(&flip_R_motor, Flip_kp_less, Flip_ki_less, Flip_kd_less);
	}
}


/**
	* @brief  Flip电流赋值函数（can1）
	* @param void
	* @retval void
	* @attention
	*/
void Set_Flip_Current()
{
	//电机目标电流为速度pid输出
	flip_R_motor.target_current = flip_R_motor.vpid.PID_OUT+500;
	flip_L_motor.target_current = flip_L_motor.vpid.PID_OUT;

	//can总线通信协议，参照电调说明书，注意，Flip有两个电机
	current_msg2[0] = flip_R_motor.target_current >> 8;			//1号电机电流高8位
	current_msg2[1] = flip_R_motor.target_current & 0xff;		//1号电机电流低8位
	current_msg2[2] = flip_L_motor.target_current >> 8;			//1号电机电流高8位
	current_msg2[3] = flip_L_motor.target_current & 0xff;		//1号电机电流低8位

	//can发送数据帧
 	CAN1_Send_Flip_Msg(current_msg2);
}
/**
	* @brief  Flip任务执行函数
	* @param void
	* @retval void
	* @attention
	*/
void Flip_Task(MOTOR_MOVE_t flip_move)
{
	
	switch(flip_move)
	{
		case out:
		{
				if(Clamp_Judge==1)
				{
					Flip_Angle_R = flip_R_motor.apid.total_angle + 250;
					Flip_Angle_L = flip_L_motor.apid.total_angle + 250;
				}
				else 
				{
					Flip_Angle_R = flip_R_motor.apid.total_angle + 40;
					Flip_Angle_L = flip_L_motor.apid.total_angle + 40;
				}
		}
		break;
		case in:
		{
				if(Clamp_Judge==1)
				{
					Flip_Angle_R = flip_R_motor.apid.total_angle - 40;
					Flip_Angle_L = flip_L_motor.apid.total_angle - 40;
				}
				else 
				{
					Flip_Angle_R = flip_R_motor.apid.total_angle - 40;
					Flip_Angle_L = flip_L_motor.apid.total_angle - 40;
				}
		}
		break;
		case stop:
		{
			Flip_Angle_R = flip_R_motor.apid.total_angle;
			Flip_Angle_L = flip_L_motor.apid.total_angle;
		}
		break;
		default:
			break;
	}
	Set_Flip_Angle_R(Flip_Angle_R);
	Set_Flip_Angle_L(Flip_Angle_L);
	Apid_Flip_Realize();
	Set_Flip_Current();
}

