#include "Remote_Task.h"
#include "BSP_Motor.h"
#include "BSP_Mineral.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"
#include "BSP_Uart.h"
#include "BSP_Chassis.h"

//内部函数声明
/**
  * @brief  遥控代码，将遥控器输出对应到机器人具体动作上，放在定时器里不断地刷
	* @param void
	* @retval void
	* @attention 应该合理设计结构，避免代码执行效率低下
  */
void Remote_Control()    //这个函数里就不断地判断每个通道的值，如果满足条件就做相应动作
{
	//底盘的控制在TIM6_Cnt_Task中
	switch(LEFT_LEVER)
	{
		case 1:
			left_act_up();//模式一
			break;
		case 2:
			left_act_down();//模式二
			break;
		case 3:
			left_act_mid();//模式三
			break;
		default:
			break;
	}
}

/**
  * @brief  左推杆拨上时候的操作（目前的操作是机械臂翻转、夹爪旋转、夹爪夹取）
	* @param void
	* @retval void
	* @attention 1.内部的方法随时替换，适应操作手 2.LEFT_LEVER==1
  */

void left_act_up()
{
	/****右拨杆控制夹爪翻转****/
	if(RIGHT_LEVER == Lever_up)
	{
	}
	if(RIGHT_LEVER == Lever_down)
	{
	}
	if(RIGHT_LEVER == Lever_mid)
	{
	}
	
	/****左摇杆左右控制机械臂伸出****/
	if(r_CH_width > 350)
	{
		Handle_Task(out);

	}
	else if(r_CH_width < -350)
	{
		Handle_Task(in);
	}	
	else
	{
		Handle_Task(stop);
	}

	
	/****左摇杆上下控制夹爪翻转****/
	if(i_CH_width > 350)
	{
		Flip_Task(in);
		Mineral_Task_Widthwise(up);
		Mineral_Task_Longitudinal(up);
	}
	else if(i_CH_width < -350)
	{
		Flip_Task(out);

	}
	else 
	{
		Flip_Task(stop);		
	}
}






/**
  * @brief  左推杆拨中时候的操作（目前的操作是纵向同步带同向转动、横向同步带异向转动、横向同步带同向转动）
	* @param void
	* @retval void
	* @attention 1.内部的方法随时替换，适应操作手 2.LEFT_LEVER==3
  */ 

void left_act_mid()
{
	Handle_Task(stop);
	Flip_Task(stop);	
	/****右拨杆控制********/
	if(RIGHT_LEVER == Lever_up)
	{
		
	}
	if(RIGHT_LEVER == Lever_down)
	{

	}
	if(RIGHT_LEVER == Lever_mid)
	{

	}
	
	/****左摇杆控制矿石翻转****/
	if(i_CH_width > 100)
	{
		Mineral_Task_Longitudinal(forward);
	}
	else if(i_CH_width < -100)
	{
		Mineral_Task_Longitudinal(back);
	}
	else if(r_CH_width > 100)
	{
		Mineral_Task_Widthwise(right);
		Mineral_Task_Longitudinal(down);
	}
	else if(r_CH_width < -100)
	{
		Mineral_Task_Widthwise(left);
		Mineral_Task_Longitudinal(down);
	}
	else if(RIGHT_LEVER == Lever_up)
	{
		Mineral_Task_Longitudinal(up);
		Mineral_Task_Widthwise(up);
	}
	else if(RIGHT_LEVER == Lever_down)
	{
		Mineral_Task_Longitudinal(down);
		Mineral_Task_Widthwise(down);
	}
	else 
	{
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
}
/**
  * @brief  左推杆拨下时候的操作（目前的操作是后电推杆、前电推杆、救援爪）
	* @param void
	* @retval void
	* @attention 1.内部的方法随时替换，适应操作手 2.LEFT_LEVER==2
  */

// 判断: 左拨杆置下
// 描述: LEFT_LEVER==2
void left_act_down()
{
	MOTOR_MOVE_t p1, p2;
	p1 = stop;
	p2 = stop;
		
		/****右拨杆控制********/
	if(RIGHT_LEVER == Lever_up)
	{
		
	}
	if(RIGHT_LEVER == Lever_down)
	{
		
	}
	if(RIGHT_LEVER == Lever_mid)
	{
		
	}
	/****左摇杆左右控制前电推杆****/
	if(r_CH_width > 200)
	{
		p1 = out;
	}
	else if(r_CH_width < -200)
	{
		p1 = in;
	}
	else
	{
		p1 = stop;
	}
	
	
	/****左摇杆上下控制后电推杆*****/
	if(i_CH_width > 200)
	{
		p2 = out;		
	}
	else if(i_CH_width < -200)
	{
		p2 = in;
	}
	else
	{
		p2 = stop;		
	}
	RC_Push_F_Send(p1);
	RC_Push_B_Send(p2);
	Handle_Task(stop);
	Flip_Task(stop);	
}

