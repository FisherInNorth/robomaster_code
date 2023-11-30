#include "chassis_task.h"
#include "supercap.h"
#include "kinematics.h"
/**
  * @breif         底盘所有任务函数，实质上是定时器中断，1ms进入一次
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */

void CHASSIS_TASK()
{
	static int time_count=1;
	time_count++;
	
	//远程遥控，调试时使用
	//remote_control();
	//chassis_control_order.chassis_mode=0;
	if(time_count%10==0)
	{
		Get_Base_Velocities();
		send_gimbal_data_2();
		//读取裁判系统
		referee_unpack_fifo_data();
		//向云台发送数据
		send_gimbal_data();
		//超级电容
	//	supercap();
	}
	
	
	if(time_count%7==0)
	{
		//底盘运动
		chassis_move();
	}
	

	
	if(time_count>=1000)			//清除计数标志    1s
	{time_count=1;}

}
