#include "RemoteChassisBoard_Task.h"

void RemoteChassisBoard(void)
{
	Load_ChassisBoard_Data();
	HAL_UART_Transmit(&huart6, Transmit_ChassisBoard_Data, 16, 1000);
}
