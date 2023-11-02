#include "steering_engine.h"
#include "remote_code.h"
#include "stm32f4xx.h"

/*
	舵机说明：
	舵机的控制通常采用 PWM 信号，例如需要一个周期为 20ms 的脉冲宽度调制（PWM），脉冲宽
	度部分一般为 0.5ms-2.5ms 范围内的角度控制脉冲部分，总间隔为 2ms。当脉冲宽度为 1.5ms 时，
	舵机旋转至中间角度，大于 1.5ms 时舵机旋转角度增大，小于 1.5ms 时舵机旋转角度减小。
*/

/***************************************舵机初始化**********************************************/

/**
  * @brief  舵机的初始化
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/* 设置输出类型 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD_COUNT; /* 累计 TIM_Period 个后产生一个更新或者中断 */
	//当定时器从 0 计数到 1000-1，即为 1000 次，为一个定时周期

	// 通用控制定时器时钟源 TIMxCLK = HCLK/2=84MHz
	// 设定定时器频率为 =TIMxCLK/1680=0.05MHz,即周期为20ms
	TIM_TimeBaseStructure.TIM_Prescaler = PWM_PRESCALER_COUNT;

	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//零分频

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 配置为 PWM 模式 1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0.5 / 20.0 * PWM_PERIOD_COUNT; //初始脉宽50
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* 当定时器计数值小于 CCR1_Val 时为高电平 */

	/* 初始化定时器 */
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
	TIM_Cmd(TIM2, ENABLE);		//使能

	/****初始化角度****/
	set_steering_gear_angle(0);
	set_steering_gear_angle_repeat(96);
}

/***************************************设置舵机角度**********************************************/

/**
  * @brief  设置舵机角度
  * @param  angle: 角度，（同舵机使用不同的角度）
  * @retval 无
	* @attention
  */
void set_steering_gear_angle_repeat(uint16_t angle_temp)
{
	angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // 计算角度对应的占空比
	set_steering_gear_dutyfactor_repeat(angle_temp);    // 设置占空比
}

/**
  * @brief  设置舵机角度
  * @param  angle: 角度，（同舵机需要使用不同的角度）
  * @retval 无
	* @attention
  */

void set_steering_gear_angle(uint16_t angle_temp)
{
	angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // 计算角度对应的占空比
	set_steering_gear_dutyfactor(angle_temp);    // 设置占空比
}


/***************************************设置舵机占空比**********************************************/

/**
  * @brief  设置舵机空比
  * @param  angle: 占空比，（0.5/20.0*PWM_PERIOD_COUNT 到 2.5/20.0*PWM_PERIOD_COUNT）
  * @retval 无
	* @attention
  */
void set_steering_gear_dutyfactor_repeat(uint16_t dutyfactor)
{

	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0.5 / 20.0 * PWM_PERIOD_COUNT > dutyfactor ? 0.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	dutyfactor = 2.5 / 20.0 * PWM_PERIOD_COUNT < dutyfactor ? 2.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	TIM_SetCompare3(TIM2, dutyfactor);
}

/**
  * @brief  设置舵机占空比
  * @param  angle: 占空比，（0.5/20.0*PWM_PERIOD_COUNT 到 2.5/20.0*PWM_PERIOD_COUNT）
  * @retval 无
  * @attention
  */
void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0.5 / 20.0 * PWM_PERIOD_COUNT > dutyfactor ? 0.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	dutyfactor = 2.5 / 20.0 * PWM_PERIOD_COUNT < dutyfactor ? 2.5 / 20.0 * PWM_PERIOD_COUNT : dutyfactor;
	TIM_SetCompare4(TIM2, dutyfactor);
}
