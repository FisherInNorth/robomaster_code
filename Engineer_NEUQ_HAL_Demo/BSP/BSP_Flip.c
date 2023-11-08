#include "BSP_Flip.h"
#include "BSP_Motor.h"
#include "BSP_Can.h"

MOTOR_MOVE_t flip_move;
MOTOR_t flip_R_motor;
MOTOR_t flip_L_motor;

int Flip_Out_Speed=20;
int Flip_In_Speed=-15;
float Flip_Kp=4.5;
float Flip_Ki=1.2;
float Flip_Kd=1.6;
int FLIP_SPEED=0;//测试用
/**
	* @function函数:Flip_Init
	* @brief描述:机械臂初始化
	* @param输入:无
	* @retval返回值:无
  */
void Flip_Init(void)
{
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
	flip_R_motor.vPID_max=3000;
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
	flip_L_motor.vPID_max=3000;
}

/**
	* @function函数:Set_Flip_Speed
	* @brief描述:机械臂伸出速度控制
	* @param输入:机械臂速度
	* @retval返回值:无
  */
void Set_Flip_Speed(int target_speed)
{
	flip_R_motor.vpid.target_speed = target_speed*19; 
	flip_L_motor.vpid.target_speed = (-target_speed)*19;
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
	* @brief  Flip电流赋值函数（can1）
	* @param void
	* @retval void
	* @attention
	*/
void Set_Flip_Current()
{
	uint8_t current_msg[8];

	//电机目标电流为速度pid输出
	flip_R_motor.target_current = flip_R_motor.vpid.PID_OUT;
	flip_L_motor.target_current = flip_L_motor.vpid.PID_OUT;
	//can总线通信协议，参照电调说明书，注意，Flip有两个电机
	current_msg[0] = flip_R_motor.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = flip_R_motor.target_current & 0xff;		//1号电机电流低8位
	current_msg[2] = flip_L_motor.target_current >> 8;			//1号电机电流高8位
	current_msg[3] = flip_L_motor.target_current & 0xff;		//1号电机电流低8位

	//can发送数据帧
 	CAN1_Send_Flip_Msg(current_msg);
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
			Set_Flip_Speed(Flip_Out_Speed);
		}
		break;
		case in:
		{
			Set_Flip_Speed(Flip_In_Speed);
		}
		break;
		case stop:
		{
			Set_Flip_Speed(0);
		}
		break;
		default:
			break;
	}
	Vpid_Realize(&flip_R_motor, Flip_Kp, Flip_Ki, Flip_Kd);
	Vpid_Realize(&flip_L_motor, Flip_Kp, Flip_Ki, Flip_Kd);
	Set_Flip_Current();
}

