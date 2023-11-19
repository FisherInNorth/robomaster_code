#include "BSP_Handle.h"
#include "BSP_Motor.h"
#include "pid.h"
#include "BSP_Can.h"
MOTOR_MOVE_t handle_move;
MOTOR_t handle_motor;
extern uint8_t current_msg2[8];
int Handle_Target_Speed=100;
float Handle_Kp=15;//80max
float Handle_Ki=1.4;
float Handle_Kd=4;

/**
	* @function函数:Handle_Init
	* @brief描述:机械臂初始化
	* @param输入:无
	* @retval返回值:无
  */
void Handle_Init(void)
{
	handle_move=stop;
	handle_motor.target_current=0;
	handle_motor.actual_current=0;
	handle_motor.round_cnt=0.0f;
	handle_motor.vpid.target_speed=0;
	handle_motor.vpid.actual_speed=0.0f;
	handle_motor.vpid.err=0;
	handle_motor.vpid.last_err=0;
	handle_motor.vpid.err_integration=0;
	handle_motor.vpid.P_OUT=0;
	handle_motor.vpid.I_OUT=0;
	handle_motor.vpid.D_OUT=0;
	handle_motor.vpid.PID_OUT=0;
	handle_motor.vPID_max=3000;
}

/**
	* @function函数:Set_Handle_Speed
	* @brief描述:机械臂伸出速度控制
	* @param输入:机械臂速度
	* @retval返回值:无
  */
void Set_Handle_Speed(int target_speed)
{
	handle_motor.vpid.target_speed = target_speed*19; 
}

/**
	* @function函数:Vpid_Handle_Realize
	* @brief描述:机械臂电机速度pid实现
	* @param输入:机械臂电机速度pid系数
	* @retval返回值:无
  */
void Vpid_Handle_Realize(float kp,float ki,float kd)
{
	Vpid_Realize(&handle_motor, kp, ki, kd);
}

/**
	* @brief  handle电流赋值函数（can1）
	* @param void
	* @retval void
	* @attention
	*/
void Set_Handle_Current()
{

	//电机目标电流为速度pid输出
	handle_motor.target_current = handle_motor.vpid.PID_OUT;

	//can总线通信协议，参照电调说明书
	current_msg2[4] = handle_motor.target_current >> 8;			//1号电机电流高8位
	current_msg2[5] = handle_motor.target_current & 0xff;		//1号电机电流低8位

	//can发送数据帧
	CAN1_Send_Handle_Msg(current_msg2);
}
/**
	* @brief  handle任务执行函数
	* @param void
	* @retval void
	* @attention
	*/
void Handle_Task(MOTOR_MOVE_t handle_move)
{
	switch(handle_move)
	{
		case out:
		{
			Set_Handle_Speed(Handle_Target_Speed);
		}
		break;
		case in:
		{
			Set_Handle_Speed(-Handle_Target_Speed);
		}
		break;
		case stop:
		{
			Set_Handle_Speed(0);
		}
		break;
		default:
			break;
	}
	
	Vpid_Handle_Realize(Handle_Kp, Handle_Ki, Handle_Kd);
	
	Set_Handle_Current();
}

