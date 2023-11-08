#include "Remote_Task.h"
#include "BSP_Motor.h"
#include "BSP_Mineral.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"


//�ڲ���������
/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
	* @param void
	* @retval void
	* @attention Ӧ�ú�����ƽṹ���������ִ��Ч�ʵ���
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
{
	switch(LEFT_LEVER)
	{
		case 1:
			left_act1();//ģʽһ
			break;
		case 2:
			left_act2();//ģʽ��
			break;
		case 3:
			left_act3();//ģʽ��
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

void left_act1()
{
	/****CH4����****/

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
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ����Ǻ���Ƹˡ�ǰ���Ƹˡ���Ԯצ��
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==2
  */

// �ж�: �󲦸�����
// ����: LEFT_LEVER==2
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
	
//��е��
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
  * @brief  ���Ƹ˲���ʱ��Ĳ�����Ŀǰ�Ĳ���������ͬ����ͬ��ת��������ͬ��������ת��������ͬ����ͬ��ת����
	* @param void
	* @retval void
	* @attention 1.�ڲ��ķ�����ʱ�滻����Ӧ������ 2.LEFT_LEVER==3
  */ 

void left_act3()
{
	
}
