#include "BSP_Chassis.h"

static void set_chassis_current(void);
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4;



/**
	* @brief  底盘任务执行函数
	* @param void
	* @retval void
	* @attention
	*/
void Chassis_Task(void)
{
  RX_chassis_speed();
	set_chassis_speed(chassis_motor1.vpid.target_speed,chassis_motor2.vpid.target_speed,chassis_motor3.vpid.target_speed,chassis_motor4.vpid.target_speed);

	//vpid_chassis_realize();

	set_chassis_current();
}


/**
  * @breif         发送四个电机的电流
  * @param[in]     none
	* @param[out]    四个电机的电流值
  * @retval        none     
  */

void set_chassis_current()
{
	uint8_t current_msg[8];

	chassis_motor1.target_current = chassis_motor1.vpid.PID_OUT;
	chassis_motor2.target_current = chassis_motor2.vpid.PID_OUT;
	chassis_motor3.target_current = chassis_motor3.vpid.PID_OUT;
	chassis_motor4.target_current = chassis_motor4.vpid.PID_OUT;

	current_msg[0] = chassis_motor1.target_current >> 8;
	current_msg[1] = chassis_motor1.target_current & 0xff;
	current_msg[2] = chassis_motor2.target_current >> 8;
	current_msg[3] = chassis_motor2.target_current & 0xff;
	current_msg[4] = chassis_motor3.target_current >> 8;
	current_msg[5] = chassis_motor3.target_current & 0xff;
	current_msg[6] = chassis_motor4.target_current >> 8;
	current_msg[7] = chassis_motor4.target_current & 0xff;

	CAN1_Chassis_Msg(current_msg);
}






