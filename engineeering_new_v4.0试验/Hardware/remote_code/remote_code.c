/**
  ******************************************************************************
  * @file    Project/HARDWARE/remote_code.c 
  * @author  Siyuan Qiao & Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention ң�������Ƴ���
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/

#include "kinematic.h"
#include "remote_code.h"
#include "motor.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "delay.h"
#include "flag.h"
#include "stepper_motor.h"
#include "bsp_dbus.h"
//�ڲ���������
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);

extern int distance;

int a =0;
uint16_t pulse_cnt = 0;   //��������������
uint16_t i = 0;           //ץȡ��ʯ���̲����

/**
  * @brief  ң�ش��룬�Զ�ȡ��
  */
void auto_remote()
{
	if()
}


/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
{	
	
	if(!LEFT_LEVER==0)			
	{
	 if(key_board==0)
	 {
		if(LEFT_LEVER == keyboard_mode)  //������������
		{
			if(RIGHT_LEVER==3)//�Ҳ�����		
			{   
			    stepper_motor_off();
				TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE );
			}						
			if(RIGHT_LEVER==1)//�Ҳ�����
			{
//				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))
//				TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
//        else
				{					
				a=0;
				TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );	
				stepper_motor_up();
				}
					
			}
			if(RIGHT_LEVER==2)//�Ҳ�����
			{
				a=0;
				TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );	
				stepper_motor_down();
			}

			Liner_X   = caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
			Liner_Y   = caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			Angular_Z = caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value, r_max_value); 

		}
		
		if(LEFT_LEVER == catch_mineral_up | LEFT_LEVER == catch_mineral_down)  //�󲦸��л�����
		{  
			Liner_X   = caculate_linear_speed(y_CH_width,y_initial_value,y_min_value,y_max_value);
			Liner_Y   = caculate_linear_speed(x_CH_width,x_initial_value,x_min_value,x_max_value);
			Angular_Z = caculate_rotational_speed(r_CH_width,r_initial_value,r_min_value, r_max_value); 
			
			if(LEFT_LEVER == catch_mineral_up)  //�󲦸���
			{
					if(i<=5)
					{
						if(RIGHT_LEVER==3 && lever_flag==0)
							lever_flag=1;
						if(RIGHT_LEVER==1 && lever_flag==1)
						{
							i++;
							lever_flag=0;
						}
					}
					if(i==1)
						handle_angle_flag=0;
					if(i==2)
						handle_angle_flag=1;
					if(i==3)
						Solenoid_handle_flag=1;
					if(i==4)
						Solenoid_valve_flag=1;					
					if(i==5)
						Solenoid_handle_flag=0;
					if(i==6)
						handle_angle_flag=0;

				
			}
			if(LEFT_LEVER == catch_mineral_down) //�󲦸���
			{
					if(i>0)
					{
						if(RIGHT_LEVER==3 && lever_flag==0)
							lever_flag=1;
						if(RIGHT_LEVER==1 && lever_flag==1)
						{
							i--;
							lever_flag=0;
						}
					}
					if(i==1)
						handle_angle_flag=2;
					if(i==2)
						handle_angle_flag=0;	
					if(i==3)
						Solenoid_valve_flag=0;
					if(i==4)
					    Solenoid_handle_flag=1;
					if(i==5)		
					    handle_angle_flag=1;
					if(i==6)		
						handle_angle_flag=0;
			}
		      if(RIGHT_LEVER == 3)    //�Ҳ�����
		       { 
					Kinematics.rescue_claw_pulse_right=120;
					Kinematics.rescue_claw_pulse_left=60;
		       }
				if(RIGHT_LEVER == 2)
				{
					Kinematics.rescue_claw_pulse_right=30;
					Kinematics.rescue_claw_pulse_left=150;
				}
		}
	}
	 else if(!key_board==0)
	{
		if(key_board&ws_key)
	     {
				if((key_board&W_key) == W_key)
					Liner_X = max_base_linear_speed;
				if((key_board&S_key) == S_key)
					Liner_X = -max_base_linear_speed;
			 }
			else
				Liner_X = 0;
			if(key_board&ad_key)
			{
				if((key_board&A_key) == A_key)
					Angular_Z = -max_base_rotational_speed;
				if((key_board&D_key) == D_key)
					Angular_Z = max_base_rotational_speed;	
			}
			else
				Angular_Z = 0;
			if(key_board&qe_key)
				{
				if((key_board&Q_key) == Q_key)
					Liner_Y = -max_base_linear_speed;
				if((key_board&E_key) == E_key)
					Liner_Y = max_base_linear_speed;	
				}
			else
				Liner_Y = 0;
/***** ����ģʽ *****/
			if((key_board&SHIFT_key) == SHIFT_key)
			{
				max_base_linear_speed = MAX_BASE_LINEAR_SPEED;
				max_base_rotational_speed = MAX_BASE_ROTATIONAL_SPEED;
			}
			else 
			{
				max_base_linear_speed = NORMAL_LINEAR_SPEED;
				max_base_rotational_speed = NORMAL_ROTATIONAL_SPEED;
			}
	}
	}	
}
	


// ����: caculate_speed()
// ����: ��ң����ҡ�����ӳ�䵽�����������ٶ���
// ������width��ͨ��ֵ 
//			 mid��ͨ���м�ֵ 
//			 min��ͨ�������Сֵ
//       max��ͨ��������ֵ
// �������Ӧ���ٶ�ֵ
//�ڲ��������û��������
static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_linear_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_linear_speed);
  else
    speed=0;
  return speed;                
}

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_rotational_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_rotational_speed);
  else
    speed=0;
  return speed;                
}

