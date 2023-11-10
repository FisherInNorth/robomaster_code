#include "BSP_Uart.h"
#include "Remote_Task.h"
#include "stdio.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
#define TX_UART huart6
#define RX_UART huart6

uint8_t RC_CH_Buffer[9];
MOTOR_MOVE_t rescue_move, rescue_card, push1, push2;
/**
   * @function函数：RC_CH_Send
   * @brief描述：发送CH通道值
   * @param输入：x,y,r,i
   * @retval返回：void
   */
void RC_CH_Send(uint16_t x,uint16_t y,uint16_t r,uint16_t i)
{
	RC_CH_Buffer[0] = '*';		//标志位
	RC_CH_Buffer[1] = (x>>8);
	RC_CH_Buffer[2] = (x&0xff);
	RC_CH_Buffer[3] = (y>>8);
	RC_CH_Buffer[4] = (y&0xff);
	RC_CH_Buffer[5] = (r>>8);
	RC_CH_Buffer[6] = (r&0xff);
	RC_CH_Buffer[7] = (i>>8);
	RC_CH_Buffer[8] = (i&0xff);
//	RC_CH_Buffer[8] = ';';
	
	HAL_UART_Transmit(&TX_UART,RC_CH_Buffer,9,0xff);
}

/**
   * @function函数：RC_Chassis_Speed_Send
   * @brief描述：发送底盘速度值
   * @param输入：x/y/r speed
   * @retval返回：void
   */
uint8_t RC_Chassis_Speed_Buffer[16];
void RC_Chassis_Speed_Send(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4)
{
	RC_Chassis_Speed_Buffer[0] =(speed1>>8);
	RC_Chassis_Speed_Buffer[1] =(speed1&0xff);
	RC_Chassis_Speed_Buffer[2] =(speed2>>8);
	RC_Chassis_Speed_Buffer[3] =(speed2&0xff);
	RC_Chassis_Speed_Buffer[4] =(speed3>>8);
	RC_Chassis_Speed_Buffer[5] =(speed3&0xff); 
	RC_Chassis_Speed_Buffer[6] =(speed4>>8);
	RC_Chassis_Speed_Buffer[7] =(speed4&0xff);
	
	HAL_UART_Transmit_DMA(&TX_UART,RC_Chassis_Speed_Buffer,16);
}

void RC_Rescue_Move_Send(MOTOR_MOVE_t rescue_move)
{
//	uint8_t TX_Rescue[4];
	switch(rescue_move)
	{
		case out:
		{
			RC_Chassis_Speed_Buffer[8] =((int)"CLA__OUT">>8);
			RC_Chassis_Speed_Buffer[9] =((int)"CLA__OUT"&0xff);
		}
		break;
		case in:
		{
			RC_Chassis_Speed_Buffer[8] =((int)"CLA___IN">>8);
			RC_Chassis_Speed_Buffer[9] =((int)"CLA___IN"&0xff);
		}	
		break;
		case stop:
		{
			RC_Chassis_Speed_Buffer[8] =((int)"CLA_STOP">>8);
			RC_Chassis_Speed_Buffer[9] =((int)"CLA_STOP"&0xff);
		}	
		break;
		
		default: break;
	}
}

void RC_Rescue_card_Send(MOTOR_MOVE_t rescue_card)
{

	switch(rescue_card)
	{
		case out:
		{
			RC_Chassis_Speed_Buffer[10] =((int)"CARD_OUT">>8);
			RC_Chassis_Speed_Buffer[11] =((int)"CARD_OUT"&0xff);
		}
		break;
		case in:
		{
			RC_Chassis_Speed_Buffer[10] =((int)"CARD__IN">>8);
			RC_Chassis_Speed_Buffer[11] =((int)"CARD__IN"&0xff);
		}	
		break;
		case stop:
		{
			RC_Chassis_Speed_Buffer[10] =((int)"CARDSTOP">>8);
			RC_Chassis_Speed_Buffer[11] =((int)"CARDSTOP"&0xff);
		}	
		break;
		
		default: break;
	}
}

void RC_Push_Send(MOTOR_MOVE_t push1, MOTOR_MOVE_t push2)
{
	switch(push1)
	{
		case out:
		{
			RC_Chassis_Speed_Buffer[12] =((int)"PUSH__QS">>8);
			RC_Chassis_Speed_Buffer[13] =((int)"PUSH__QS"&0xff);
		}
		break;
		case in:
		{
			RC_Chassis_Speed_Buffer[12] =((int)"PUSH__HS">>8);
			RC_Chassis_Speed_Buffer[13] =((int)"PUSH__HS"&0xff);
		}	
		break;
	
		default: break;
	}
		switch(push2)
	{
		case out:
		{
			RC_Chassis_Speed_Buffer[14] =((int)"PUSH__QJ">>8);
			RC_Chassis_Speed_Buffer[15] =((int)"PUSH__QJ"&0xff);
		}
		break;
		case in:
		{
			RC_Chassis_Speed_Buffer[14] =((int)"PUSH__HJ">>8);
			RC_Chassis_Speed_Buffer[15] =((int)"PUSH__HJ"&0xff);
		}	
		break;
	
		default: break;
	}
}




/**
   * @function函数：Usart_SendString
   * @brief描述：发送字符串
   * @param输入：str
   * @retval返回：void
   */
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(&TX_UART,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}
uint8_t RX_Bufer[50];
uint32_t RecCount=0;

//空闲中断初始化
void UART_IdleIT_init()
{
//	__HAL_UART_ENABLE_IT(&RX_UART,UART_IT_IDLE);
//	__HAL_UART_CLEAR_IDLEFLAG(&RX_UART);	
//	HAL_UART_Receive_DMA(&RX_UART,RX_Bufer,50);
	HAL_UART_Receive_IT(&RX_UART,RX_Bufer,4);
}
uint8_t RX_temp[1]={1};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit_IT(&TX_UART,(uint8_t*)"1234",4);
	HAL_UART_Receive_IT(&RX_UART,RX_Bufer,4);
}
//void USART6_IRQHandler(void)
//{
//	
////	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET)
////	{
////		__HAL_UART_CLEAR_IDLEFLAG(&RX_UART);
////    HAL_UART_DMAStop(&RX_UART);
////		RecCount=50-__HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
////		HAL_UART_Transmit_DMA(&TX_UART,RX_Bufer,4);
////		RecCount=0;
////		HAL_UART_Receive_DMA(&RX_UART,RX_Bufer,50);
////	}
//}
//void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
//{
//	RecCount=50-__HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
//	HAL_UART_Transmit_DMA(&TX_UART,RX_Bufer,RecCount);
//	RecCount=0;
//	HAL_UART_Receive_DMA(&RX_UART,RX_Bufer,50);

//}
	
///**
//  * @brief This function handles USART1 global interrupt.
//  */
//void USART3_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART1_IRQn 0 */
//	if(__HAL_UART_GET_FLAG(&huart3,UART_IT_IDLE)==1)
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//		HAL_UART_DMAStop(&huart1);
//		HAL_UART_IdleCpltCallback(&huart1);
//		
//	}
//  /* USER CODE END USART1_IRQn 0 */
//  HAL_UART_IRQHandler(&huart1);
//  /* USER CODE BEGIN USART1_IRQn 1 */

//  /* USER CODE END USART1_IRQn 1 */
//}


