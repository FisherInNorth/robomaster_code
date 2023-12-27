/**
  ******************************************************************************
  * @file    Project/USER/main.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
      ..................NEUQ_SUDO..................

  ...........������δ�ɹ���ͬ־����Ŭ��...........
*/  
#include "My_Init.h"

extern uint16_t pulse_cnt;   //��������������
extern int a;
/**
  *@brief  ��������ʼ��������ѭ���ȴ��ж�
  */
int main()
{
	//software_init(); //������ṹ���ʼ��
	//hardware_init(); //Ӳ����ʼ��
	All_Init();
	//TIM_SetCompare3(TIM2, 150);
	while(1)                        
	{
		
	}
}

/**
  * @brief  ��ʱ��3�жϷ�����			1ms
  */
void TIM3_IRQHandler(void)
{
	static int time_count=1;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 	//����ж�
	{ 
		  time_count++;
		
		/*****   ң��������    ******/
		  Remote_Control();				//ң�������ƴ���		
			
		/****  �������˶�����  *****/
		if(time_count%7 ==0)		//7ms
			Robo_Move();
		/*****    ����ɨ��   ******/
		if(time_count%31 == 0)		//31ms  ����
			Debug_Key();
		/*****    ����״ָ̬ʾ��   ******/
		if(time_count%500 == 0)			//500ms
		{
			LED1=!LED1;							//��Ƭ����������״ָ̬ʾ��
			time_count = 0;
		}
		if(time_count%4 == 0)		//4ms  ����
			Get_Base_Velocities();		//�����������ʵ���ٶ�
		   
		if(time_count>=1000)			//���������־    1s
			time_count=1;
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}



void TIM4_IRQHandler(void)
{
	if(a==0)
	{
		
		if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 	//����ж�
			pulse_cnt++;
	}
	if(a==1)
	{
		if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 	//����ж�
			pulse_cnt--;
	}
	
	
	

    limit_switch.count++;
    if(limit_switch.count == stepper_motor_left.pwm_num)
    {
		limit_switch.count = 0;
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
    }

	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //����жϱ�־λ
}
