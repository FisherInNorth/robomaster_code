#include "UpperBoard.h"
#include "ReceiveUpperBoard_Task.h"

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //���ڽ����жϻص�����
//{
//	LED_Blue(1);
//	if(huart->Instance == USART1)   //�жϷ��������жϵĴ���
//	{
//		
//	}
//}

void ReceiveUpperBoard(unsigned char data)
{
	pUpperBoard_DataH ++;
	if(pUpperBoard_DataH >= 100)
	{
		pUpperBoard_DataH = 0;
		
	}
	Receive_UpperBoard_Data[pUpperBoard_DataH] = data;
	//Resolve_ChassisBoard_Data();
}


//https://blog.csdn.net/weixin_49821504/article/details/126848758
