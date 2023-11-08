#include "Remote_Task.h"
#include "BSP_Motor.h"
#include "BSP_Mineral.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"


//内部函数声明
/**
  * @brief  遥控代码，将遥控器输出对应到机器人具体动作上，放在定时器里不断地刷
	* @param void
	* @retval void
	* @attention 应该合理设计结构，避免代码执行效率低下
  */
void Remote_Control()    //这个函数里就不断地判断每个通道的值，如果满足条件就做相应动作
{
	switch(LEFT_LEVER)
	{
		case 1:
			left_act1();//模式一
			break;
		case 2:
			left_act2();//模式二
			break;
		case 3:
			left_act3();//模式三
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

void left_act1()
{
	/****CH4控制****/

	if(i_CH_width > 100){
		Mineral_Task_Longitudinal(forward);
	}
	else if(i_CH_width < -100){
		Mineral_Task_Longitudinal(back);
	}
	else if(r_CH_width > 100){
		Mineral_Task_Widthwise(right);
		Mineral_Task_Longitudinal(down);
	}
	else if(r_CH_width < -100){
		Mineral_Task_Widthwise(left);
		Mineral_Task_Longitudinal(down);
	}
	else if(x_CH_width > 100)
	{
		Mineral_Task_Longitudinal(up);
		Mineral_Task_Widthwise(up);
	}

	else if(x_CH_width < -100)
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
void left_act2()
{

	if(RIGHT_LEVER == 1)
	{
		Handle_Task(out);
	}
	if(RIGHT_LEVER == 2)
	{
		Handle_Task(in);
	}
	if(RIGHT_LEVER == 3)
	{
		Handle_Task(stop);
	}
	
//机械臂
	if(y_CH_width > 100)
	{
		Flip_Task(out);
	}
	else if(y_CH_width < -100)
	{
		Flip_Task(in);
	}
	else if(y_CH_width <= 100 || y_CH_width >= -100)
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

void left_act3()
{
	
}
