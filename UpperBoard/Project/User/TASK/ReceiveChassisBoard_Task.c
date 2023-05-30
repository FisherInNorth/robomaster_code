#include "ReceiveChassisBoard_Task.h"

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //串口接收中断回调函数
//{
//	LED_Blue(1);
//	if(huart->Instance == USART1)   //判断发生接收中断的串口
//	{
//		
//	}
//}

void ReceiveChassisBoard(unsigned char data)
{
	pChassisBoard_DataH ++;
	if(pChassisBoard_DataH >= 100)
	{
		pChassisBoard_DataH = 0;
		
	}
	ChassisBoard_Data[pChassisBoard_DataH] = data;
	//Resolve_ChassisBoard_Data();
}


//https://blog.csdn.net/weixin_49821504/article/details/126848758
