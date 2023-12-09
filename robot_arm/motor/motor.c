/**
  ******************************************************************************
  * @file    Project/HARDWARE/motor.c
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief
  ******************************************************************************
  * @attention
  ******************************************************************************
	// * 范围  ：3508  数据范围                       金属头转速为0.22rps,即13.3rpm
//						给定电流 -16384 ~ 0 ~ +16384
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=450	RPM
//						实际转矩电流
//						电机温度
//			   ：GM6020数据范围
//						给定电压 -30000 ~ 0 ~ +30000
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=350	RPM
//						实际转矩电流
//						电机温度
//			   ：P2006 数据范围
//						给定电流 -10000 ~ 0 ~ +10000
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=450	RPM
//						实际输出转矩
      ..................NEUQ_SUDO..................

*/
#include "motor.h"
#include "pid.h" 

/**
* @brief  电机can回调赋值函数
* @param MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current
* @retval void
* @attention
*/
void record_motor_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->apid.actual_angle;
	motor->apid.actual_angle = angle;
	motor->vpid.actual_speed = speed;
	motor->actual_current = current;
	if(motor->start_angle_flag == 0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}

	if(motor->apid.actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->apid.actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}

/**
* @brief  所有电机参数初始化
* @param 
* @retval 
* @attention
*/
void motor_Init()
{
	motor_all_init(&motor1);
	motor_all_init(&motor2);
	motor_all_init(&motor3);
	motor_all_init(&motor4);
	motor_all_init(&motor5);
	motor_all_init(&motor6);
	motor_all_init(&motor7);
}


/**
* @brief  所有电机参数初始化
* @param 
* @retval 
* @attention
*/
void motor_all_init(MOTOR_t *motor)
{
	//电机初始化 
	motor->start_angle = 0;
	motor->apid.actual_angle = 0;
	motor->vpid.actual_speed = 0;
	motor->start_angle_flag = 0;
	motor->actual_current = 0;
	motor->target_current = 0;
}

/**
* @brief  chassis电流赋值函数（can1）
* @param void
* @retval void
* @attention
*/
void set_chassis_current()
{
	u8 current_msg[8];

	//电机目标电流为速度pid输出
	motor1.target_current = motor1.vpid.PID_OUT;
	motor2.target_current = motor2.vpid.PID_OUT;
	motor3.target_current = motor3.vpid.PID_OUT;
	motor4.target_current = motor4.vpid.PID_OUT;


	//can总线通信协议，参照电调说明书
	current_msg[0] = motor1.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = motor1.target_current & 0xff;		//1号电机电流低8位
	current_msg[2] = motor2.target_current >> 8;			//2号电机电流高8位
	current_msg[3] = motor2.target_current & 0xff;		//2号电机电流低8位
	current_msg[4] = motor3.target_current >> 8;			//3号电机电流高8位
	current_msg[5] = motor3.target_current & 0xff;		//3号电机电流低8位
	current_msg[6] = motor4.target_current >> 8;			//4号电机电流高8位
	current_msg[7] = motor4.target_current & 0xff;		//4号电机电流低8位

	//can发送数据帧
	CAN1_Send_CHASSIS_Msg(current_msg);
}

/**
* @brief  handle电流赋值函数（can1）
* @param void
* @retval void
* @attention
*/
void set_rescuecard_current()
{
	u8 current_msg[8];

	//电机目标电流为速度pid输出
	motor5.target_current = motor5.vpid.PID_OUT;

	//can总线通信协议，参照电调说明书
	current_msg[0] = motor5.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = motor5.target_current & 0xff;		//1号电机电流低8位

	//can发送数据帧
	CAN1_Send_rescuecard_Msg(current_msg);
}

/**
* @brief  flip电流赋值函数（can1）
* @param void
* @retval void
* @attention
*/
void set_rescueclaw_current()
{
	u8 current_msg[8];

	//电机目标电流为速度pid输出
	motor6.target_current = motor6.vpid.PID_OUT;
	motor7.target_current = motor7.vpid.PID_OUT;

	//can总线通信协议，参照电调说明书
	current_msg[2] = motor6.target_current >> 8;			//1号电机电流高8位
	current_msg[3] = motor6.target_current & 0xff;		//1号电机电流低8位
	current_msg[4] = motor7.target_current >> 8;			//1号电机电流高8位
	current_msg[5] = motor7.target_current & 0xff;		//1号电机电流低8位

	//使用can1发送数据帧
	CAN1_Send_rescueclaw_Msg(current_msg);
}

