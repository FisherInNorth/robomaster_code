#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "usart.h"

unsigned char UART_RX_Buffer[18];

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	//HAL_UART_Transmit_IT(&TX_UART,(uint8_t*)"1234",4);
//	
//	//HAL_UART_Transmit(&huart6,UART_RX_Buffer,16,1000);

//	//HAL_UART_Receive_IT(&huart1,UART_RX_Buffer,18);
//}

