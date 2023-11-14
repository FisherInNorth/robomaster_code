#include "Remote_Task.h"
#include "BSP_Motor.h"
#include "BSP_Mineral.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"
#include "BSP_Uart.h"
#include "BSP_Chassis.h"
#include "BSP_Electric_push.h"

extern uint8_t Keyboard_Mode;

//�ڲ���������
/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
	* @param void
	* @retval void
	* @attention Ӧ�ú�����ƽṹ���������ִ��Ч�ʵ���
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
{
	if(Keyboard_Mode==0)
	{
		//���̵Ŀ�����TIM6_Cnt_Task��
		switch(LEFT_LEVER)
		{
			case 1:
				left_act_up();//ģʽһ
				break;
			case 2:
				left_act_down();//ģʽ��
				break;
			case 3:
				left_act_mid();//ģʽ��
				break;
			default:
				break;
		}
	}
}


/**
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ����ǻ�е�۷�ת����צ��ת����צ��ȡ��
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==1
  */

void left_act_up()
{

	/****�Ҳ��˿��Ƽ�צ��ȡ****/
	if(RIGHT_LEVER == Lever_up)
	{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);		
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(RIGHT_LEVER == Lever_down)
	{
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);		
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(RIGHT_LEVER == Lever_mid)
	{
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);		
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	/****��ҡ�����ҿ��ƻ�е�����****/
	if(r_CH_width > 350)
	{
		Handle_Task(in);
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}
	else if(r_CH_width < -350)
	{
		Handle_Task(out);
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);
	}	
	else if(r_CH_width <= 350 && r_CH_width >= -350)
	{
		Handle_Task(stop);
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);	
	}

	
	/****��ҡ�����¿��Ƽ�צ��ת****/
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
	else 
	{
		Flip_Task(stop);	
		Mineral_Task_Longitudinal(stop);
		Mineral_Task_Widthwise(stop);		
	}
}




/**
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ���������ͬ����ͬ��ת��������ͬ��������ת��������ͬ����ͬ��ת����
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==3
  */ 

void left_act_mid()
{
	Handle_Task(stop);
	Flip_Task(stop);	
	/****�Ҳ��˿���********/
	if(RIGHT_LEVER == Lever_up)
	{
		
	}
	if(RIGHT_LEVER == Lever_down)
	{

	}
	if(RIGHT_LEVER == Lever_mid)
	{

	}
	
	/****��ҡ�˿��ƿ�ʯ��ת****/
	if(i_CH_width > 640)
	{
		Mineral_Task_Longitudinal(forward);
	}
	else if(i_CH_width < -640)
	{
		Mineral_Task_Longitudinal(back);
	}
	else if(r_CH_width > 640)
	{
		Mineral_Task_Widthwise(right);
	}
	else if(r_CH_width < -640)
	{
		Mineral_Task_Widthwise(left);
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
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ����Ǻ���Ƹˡ�ǰ���Ƹˡ���Ԯצ��
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==2
  */

// �ж�: �󲦸�����
// ����: LEFT_LEVER==2
void left_act_down()
{
	Mineral_Task_Longitudinal(stop);
	Mineral_Task_Widthwise(stop);
	MOTOR_MOVE_t p1, p2;
	p1 = stop;
	p2 = stop;
		
		/****�Ҳ��˿���********/
	if(RIGHT_LEVER == Lever_up)
	{
		
	}
	if(RIGHT_LEVER == Lever_down)
	{
		
	}
	if(RIGHT_LEVER == Lever_mid)
	{
		
	}
	/****��ҡ�����ҿ���ǰ���Ƹ�****/
	if(r_CH_width > 200)
	{
		p1 = qs;
	}
	else if(r_CH_width < -200)
	{
		p1 = qj;
	}
	else
	{
		p1 = qt;
	}
	
	
	/****��ҡ�����¿��ƺ���Ƹ�*****/
	if(i_CH_width > 200)
	{
		p2 = hs;		
	}
	else if(i_CH_width < -200)
	{
		p2 = hj;
	}
	else
	{
		p2 = ht;		
	}
  Push_Task(p1, p2);
	Handle_Task(stop);
	Flip_Task(stop);	
}

