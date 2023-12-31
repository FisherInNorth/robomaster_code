#include "Remote_Task.h"
#include "BSP_Motor.h"
#include "BSP_Mineral.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"
#include "BSP_Uart.h"
#include "BSP_Chassis.h"
#include "BSP_Electric_push.h"
#include "BSP_Clamp.h"
extern uint8_t Keyboard_Mode;
extern uint8_t Flip_PID2_Flag;
//内部函数声明
/**
  * @brief  遥控代码，将遥控器输出对应到机器人具体动作上，放在定时器里不断地刷
	* @param void
	* @retval void
	* @attention 应该合理设计结构，避免代码执行效率低下
  */
void Remote_Control()    //这个函数里就不断地判断每个通道的值，如果满足条件就做相应动作
{
	if((RIGHT_LEVER==Lever_up) || (RIGHT_LEVER==Lever_down) || x_CH_width || y_CH_width || r_CH_width || i_CH_width || DJI_Motion_Round )
	{
		Keyboard_Mode = 0;
	}
	else
	{
		Keyboard_Mode = 1;
	}
	if(Keyboard_Mode==0)
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
}


/**
  * @brief  左推杆拨上时候的操作（目前的操作是机械臂翻转、夹爪旋转、夹爪夹取）
	* @param void
	* @retval void
	* @attention 1.内部的方法随时替换，适应操作手 2.LEFT_LEVER==1
  */

void left_act_up()
{
	RC_Rescue_card_Send(stop);
	RC_Rescue_Move_Send(stop);
	/****右拨杆控制夹爪夹取****/
	if(RIGHT_LEVER == Lever_up && (i_CH_width <= 350))
	{
    Clamp_ON; Flip_PID2_Flag=1;
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(RIGHT_LEVER == Lever_mid && (i_CH_width <= 350))
	{
		Clamp_OFF; Flip_PID2_Flag=0;
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(RIGHT_LEVER == Lever_down && (i_CH_width <= 350))
	{
		AirMineral_Task();
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);		
	}

	/****左摇杆左右控制机械臂伸出****/
	if(r_CH_width > 350 && (i_CH_width <= 350))
	{
		Handle_Task(in);
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(r_CH_width < -350 && (i_CH_width <= 350))
	{
		Handle_Task(out);
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}	
	else if(r_CH_width <= 350 && r_CH_width >= -350 && (i_CH_width <= 350))
	{
		Handle_Task(stop);
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
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
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(i_CH_width <= 350 && i_CH_width >= -350)
	{
		Flip_Task(stop);	
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
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
	RC_Rescue_Move_Send(stop);
	

	/****左摇杆控制矿石翻转****/
	if(i_CH_width > 640)
	{
		Mineral_Task_Longitudinal(forward);
	}
	if(i_CH_width < -640)
	{
		Mineral_Task_Longitudinal(back);
	}
	if(i_CH_width <= 640 && i_CH_width >= -640 && (RIGHT_LEVER == Lever_mid))
	{
		Mineral_Task_Longitudinal(stop);
	}
	
	if(r_CH_width > 640)
	{
		Mineral_Task_Widthwise(right);
	}
	if(r_CH_width < -640)
	{
		Mineral_Task_Widthwise(left);
	}
	if(r_CH_width >= -640 && r_CH_width < 640 && (RIGHT_LEVER == Lever_mid))
	{
		Mineral_Task_Widthwise(stop);
	}
		/****右拨杆控制********/
	if(RIGHT_LEVER == Lever_up)
	{
		Mineral_Task_Longitudinal(up);
		Mineral_Task_Widthwise(up);
	}
	else if(RIGHT_LEVER == Lever_down)
	{
		Mineral_Task_Longitudinal(down);
		Mineral_Task_Widthwise(down);
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
	Mineral_Task_Longitudinal(stop);
	Mineral_Task_Widthwise(stop);
	RC_Rescue_card_Send(stop);
	MOTOR_MOVE_t Push_F, Push_B;
	MOTOR_MOVE_t rescue_move;
	Push_F = stop;
	Push_B = stop;
	
		
		/****右拨杆控制********/
	if(RIGHT_LEVER == Lever_up)
	{
		rescue_move = out;
	}
	else if(RIGHT_LEVER == Lever_down)
	{
		rescue_move = in;
	}
	else if(RIGHT_LEVER == Lever_mid)
	{
		rescue_move = stop;
	}
	/****左摇杆左右控制前电推杆****/
	if(r_CH_width > 200)
	{
		Push_F = qs;
	}
	else if(r_CH_width < -200)
	{
		Push_F = qj;
	}
	else
	{
		Push_F = qt;
	}
	
	
	/****左摇杆上下控制后电推杆*****/
	if(i_CH_width > 200)
	{
		Push_B = hs;		
	}
	else if(i_CH_width < -200)
	{
		Push_B = hj;
	}
	else
	{
		Push_B = ht;		
	}
  Push_Task(Push_F, Push_B);
	Handle_Task(stop);
	Flip_Task(stop);	
	RC_Rescue_Move_Send(rescue_move);
}

