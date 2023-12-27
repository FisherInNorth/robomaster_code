#include "key.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

// ����: key_init()
// ����: ����GPIO��ʼ��
// ����: ��
void key_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;		//����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;		//����
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
}

// ����: key_press()
// ����: �жϰ����Ƿ񱻰���
// ����: ��
// return�� 1Ϊ���£�0Ϊ����
u8 key_press()
{
	static u8 key_state = 0,key_last_state = 0;
	u8 data_back = 0;
	
	key_state = PBin(2);		 //����ǰ����״̬
	if(key_state==1)
	{
		if(key_last_state == key_state)		//�������״̬��ͬ���Ǹߵ�ƽ��˵������������
		{
			key_last_state = key_state;
			data_back = 1;
		}
		else
		{
			key_last_state = key_state;
			data_back = 0;
		}
	}
	else
		data_back = 0;
	
	return data_back;
		
}



