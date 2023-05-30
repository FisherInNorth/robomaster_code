#include "RemoteUpperBoard_Task.h"

void RemoteUpperBoard(void)
{
	Load_UpperBoard_Data();
	HAL_UART_Transmit(&huart6, UpperBoard_Data, 16, 1000);
}
