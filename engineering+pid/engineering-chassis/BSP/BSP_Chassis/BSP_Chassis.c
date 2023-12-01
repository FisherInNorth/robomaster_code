#include "BSP_Chassis.h"

static void chassis_move(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static void set_chassis_current(void);
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4;
CHASSIS_CONTROL_ORDER_t chassis_control_order;



/**
	* @brief  ��������ִ�к���
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
  * @breif         ��ȡ���̵������ٶ�
	* @param[out]    �������ĵ����������ٶ�
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
  * @breif         �˶�ѧ�ֽ⣬���������ĵ��ٶ�ת��Ϊ�ĸ����ӵ��ٶ�
  * @param[in]     speed_x��x�����ٶ�
	* @param[in]     speed_y��y�����ٶ�
	* @param[in]     speed_r����ת�ٶ�
	* @param[out]    �ĸ������Ŀ���ٶ�
  * @retval        none     
  */
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
	//�ٶȻ��㣬�˶�ѧ�ֽ�
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
  * @breif         �����ĸ�����ĵ���
  * @param[in]     none
	* @param[out]    �ĸ�����ĵ���ֵ
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






