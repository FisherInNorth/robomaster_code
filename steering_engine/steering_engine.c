#include "steering_engine.h"
#include "remote_code.h"
#include "stm32f4xx.h"

/*
	���˵����
	����Ŀ���ͨ������ PWM �źţ�������Ҫһ������Ϊ 20ms �������ȵ��ƣ�PWM���������
	�Ȳ���һ��Ϊ 0.5ms-2.5ms ��Χ�ڵĽǶȿ������岿�֣��ܼ��Ϊ 2ms����������Ϊ 1.5ms ʱ��
	�����ת���м�Ƕȣ����� 1.5ms ʱ�����ת�Ƕ�����С�� 1.5ms ʱ�����ת�Ƕȼ�С��
*/

/***************************************�����ʼ��**********************************************/

/**
  * @brief  ����ĳ�ʼ��
	* @retval void
	* @attention
  */
void TIM_Steering_Engine_PWM_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/* ����������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_COUNT; /* �ۼ� TIM_Period �������һ�����»����ж� */
	//����ʱ���� 0 ������ 1000-1����Ϊ 1000 �Σ�Ϊһ����ʱ����

	// ͨ�ÿ��ƶ�ʱ��ʱ��Դ TIMxCLK = HCLK/2=84MHz
	// �趨��ʱ��Ƶ��Ϊ =TIMxCLK/1680=0.05MHz,������Ϊ20ms
	TIM_TimeBaseStructure.TIM_Prescaler = PWM_PRESCALER_COUNT;

	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//���Ƶ

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ����Ϊ PWM ģʽ 1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0.5 / 20.0 * PWM_PERIOD_COUNT; //��ʼ����50
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* ����ʱ������ֵС�� CCR1_Val ʱΪ�ߵ�ƽ */

	/* ��ʼ����ʱ�� */
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);		//ʹ��

	/****��ʼ���Ƕ�****/
	set_steering_gear_angle(0);
	set_steering_gear_angle_repeat(96);
}

/***************************************���ö���Ƕ�**********************************************/

/**
  * @brief  ���ö���Ƕ�
  * @param  angle: �Ƕȣ���ͬ���ʹ�ò�ͬ�ĽǶȣ�
  * @retval ��
	* @attention
  */
void set_steering_gear_angle_repeat(uint16_t angle_temp)
{
	angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // ����Ƕȶ�Ӧ��ռ�ձ�
	set_steering_gear_dutyfactor_repeat(angle_temp);    // ����ռ�ձ�
}

/**
  * @brief  ���ö���Ƕ�
  * @param  angle: �Ƕȣ���ͬ�����Ҫʹ�ò�ͬ�ĽǶȣ�
  * @retval ��
	* @attention
  */

void set_steering_gear_angle(uint16_t angle_temp)
{
	angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // ����Ƕȶ�Ӧ��ռ�ձ�
	set_steering_gear_dutyfactor(angle_temp);    // ����ռ�ձ�
}


/***************************************���ö��ռ�ձ�**********************************************/

/**
  * @brief  ���ö���ձ�
  * @param  angle: ռ�ձȣ���0.5/20.0*PWM_PERIOD_COUNT �� 2.5/20.0*PWM_PERIOD_COUNT��
  * @retval ��
	* @attention
  */
void set_steering_gear_dutyfactor_repeat(uint16_t dutyfactor)
{

	/* �Գ�����Χ��ռ�ձȽ��б߽紦�� */
	dutyfactor = 0.5 / 20.0 * PWM_PERIOD_COUNT > dutyfactor ? 0.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	dutyfactor = 2.5 / 20.0 * PWM_PERIOD_COUNT < dutyfactor ? 2.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	TIM_SetCompare3(TIM2, dutyfactor);
}

/**
  * @brief  ���ö��ռ�ձ�
  * @param  angle: ռ�ձȣ���0.5/20.0*PWM_PERIOD_COUNT �� 2.5/20.0*PWM_PERIOD_COUNT��
  * @retval ��
  * @attention
  */
void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
	/* �Գ�����Χ��ռ�ձȽ��б߽紦�� */
	dutyfactor = 0.5 / 20.0 * PWM_PERIOD_COUNT > dutyfactor ? 0.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	dutyfactor = 2.5 / 20.0 * PWM_PERIOD_COUNT < dutyfactor ? 2.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	TIM_SetCompare4(TIM2, dutyfactor);
}
