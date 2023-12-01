#include "BSP_Chassis.h"

static void chassis_move(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static void set_chassis_current(void);
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4;
CHASSIS_CONTROL_ORDER_t chassis_control_order;



/**
	* @brief  底盘任务执行函数
	* @param void
	* @retval void
	* @attention
	*/
void Chassis_Task(void)
{

	chassis_move();
	
	//vpid_chassis_realize();

	set_chassis_current();
}


float vx,vy,wz;
/**
  * @breif         获取底盘的中心速度
	* @param[out]    底盘中心的三个方向速度
  * @retval        none     
  */
static void chassis_move(void)
{
	vx=(float)chassis_control_order.vx_set;
	vy=(float)chassis_control_order.vy_set;
	wz=(float)chassis_control_order.wz_set;
	
	chassis_speed_control(vx,vy,wz);
}

/**
  * @breif         运动学分解，将底盘中心的速度转换为四个轮子的速度
  * @param[in]     speed_x：x方向速度
	* @param[in]     speed_y：y方向速度
	* @param[in]     speed_r：自转速度
	* @param[out]    四个电机的目标速度
  * @retval        none     
  */
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
	//速度换算，运动学分解
	BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
	
	max=find_max();
	if(max>MAX_MOTOR_SPEED)
	{
		chassis_motor1.vpid.target_speed=(int)(chassis_motor1.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor2.vpid.target_speed=(int)(chassis_motor2.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor3.vpid.target_speed=(int)(chassis_motor3.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor4.vpid.target_speed=(int)(chassis_motor4.vpid.target_speed*MAX_MOTOR_SPEED*1.0/max);
	}
	set_chassis_speed(chassis_motor1.vpid.target_speed, chassis_motor2.vpid.target_speed, chassis_motor3.vpid.target_speed, chassis_motor4.vpid.target_speed);
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

	//CAN1_Send_CHASSIS_Msg(current_msg);
}






