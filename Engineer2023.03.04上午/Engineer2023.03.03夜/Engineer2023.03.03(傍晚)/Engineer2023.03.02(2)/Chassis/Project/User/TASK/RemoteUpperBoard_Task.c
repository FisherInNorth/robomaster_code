#include "RemoteUpperBoard_Task.h"

void RemoteUpperBoard(void)
{
	Load_UpperBoard_Data();
	HAL_UART_Transmit_IT(&huart1,UpperBoard_Data,8);
}

