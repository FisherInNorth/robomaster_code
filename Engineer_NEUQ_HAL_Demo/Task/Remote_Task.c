#include "Remote_Task.h"
#include "BSP_Motor.h"
#include "BSP_Mineral.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"
#include "BSP_Uart.h"
#include "BSP_Chassis.h"

//�ڲ���������
/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
	* @param void
	* @retval void
	* @attention Ӧ�ú�����ƽṹ���������ִ��Ч�ʵ���
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
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

/**
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ����ǻ�е�۷�ת����צ��ת����צ��ȡ��
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==1
  */

void left_act_up()
{
	/****�Ҳ��˿��Ƽ�צ��ת****/
	if(RIGHT_LEVER == Lever_up)
	{
	}
	if(RIGHT_LEVER == Lever_down)
	{
	}
	if(RIGHT_LEVER == Lever_mid)
	{
	}
	
	/****��ҡ�����ҿ��ƻ�е�����****/
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

	}
	else 
	{
		Flip_Task(stop);		
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
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ����Ǻ���Ƹˡ�ǰ���Ƹˡ���Ԯצ��
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==2
  */

// �ж�: �󲦸�����
// ����: LEFT_LEVER==2
void left_act_down()
{
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
	
	
	/****��ҡ�����¿��ƺ���Ƹ�*****/
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

