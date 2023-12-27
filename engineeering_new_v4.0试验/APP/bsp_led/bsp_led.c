/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ledӦ�ú����ӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F429 ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_led.h"   

 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
	
void LED_Black(void)
{
	LED1_ON;
	LED2_ON;
	LED3_ON;
	LED4_ON;
	LED5_ON;
	LED6_ON;
	LED7_ON;
	LED8_ON;
}
void LED_Bright(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	LED5_OFF;
	LED6_OFF;
	LED7_OFF;
	LED8_OFF;
}
void LED_Toggle(void)
{
	LED1_TOGGLE;
	LED2_TOGGLE;
	LED3_TOGGLE;
	LED4_TOGGLE;
	LED5_TOGGLE;
	LED6_TOGGLE;
	LED7_TOGGLE;
	LED8_TOGGLE;
}
void LED_Control(uint8_t select)
{
	LED1(select&(0x01));
	LED2(select&(0x02));
	LED3(select&(0x04));
	LED4(select&(0x08));
	LED5(select&(0x10));
	LED6(select&(0x20));
	LED7(select&(0x40));
	LED8(select&(0x80));
}
void LED_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����LED��ص�GPIO����ʱ��*/
		RCC_AHB1PeriphClockCmd (LED_GPIO_CLK|LEDG_GPIO_CLK|LEDR_GPIO_CLK|LASER_GPIO_CLK, ENABLE); 

		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED1_PIN;	

		/*��������ģʽΪ���ģʽ*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
    
    /*�������ŵ��������Ϊ�������*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    
    /*��������Ϊ����ģʽ��Ĭ��LED��*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 

		/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	
    
    /*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED2_PIN;	
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);	
    
    /*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED3_PIN;	
    GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED4_PIN;	
    GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED5_PIN;	
    GPIO_Init(LED5_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED6_PIN;	
    GPIO_Init(LED6_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED7_PIN;	
    GPIO_Init(LED7_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LED8_PIN;	
    GPIO_Init(LED8_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LEDG_PIN;	
    GPIO_Init(LEDG_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LEDR_PIN;	
    GPIO_Init(LEDR_GPIO_PORT, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/															   
		GPIO_InitStructure.GPIO_Pin = LASER_PIN;	
    GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStructure);	

		/*�ر�LED��*/
		LED_Black();
		LEDG_OFF;LEDR_ON;LASER_OFF
}
/*********************************************END OF FILE**********************/
