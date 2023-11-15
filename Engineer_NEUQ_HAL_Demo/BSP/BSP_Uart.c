#include "BSP_Uart.h"
#include "Remote_Task.h"
#include "stdio.h"

int send_check = 0;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
#define TX_UART huart6
#define RX_UART huart6

uint8_t RC_CH_Buffer[9];
MOTOR_MOVE_t rescue_move, rescue_card, push1, push2;

////红外控制电磁阀
//void Valve_Task(void)
//{
//	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0))
//	{HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);}
//	else HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET);
//}

/**
   * @function函数：RC_Chassis_Speed_Send
   * @brief描述：发送底盘速度值
   * @param输入：x/y/r speed
   * @retval返回：void
   */
uint8_t RC_Chassis_Speed_Buffer[12] = {0, 0, 0, 0, 0, 0, 0, 0, 'p', 'p','p', 'p'};
uint8_t Rescue_Push_Buffer[4] = {0, 0, 0, 0};
void RC_Chassis_Speed_Send(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4)
{
	RC_Chassis_Speed_Buffer[0] = (speed1>>8);
	RC_Chassis_Speed_Buffer[1] = (speed1&0xff);
	RC_Chassis_Speed_Buffer[2] = (speed2>>8);
	RC_Chassis_Speed_Buffer[3] = (speed2&0xff);
	RC_Chassis_Speed_Buffer[4] = (speed3>>8);
	RC_Chassis_Speed_Buffer[5] = (speed3&0xff); 
	RC_Chassis_Speed_Buffer[6] = (speed4>>8);
	RC_Chassis_Speed_Buffer[7] = (speed4&0xff);
	if(send_check == 0)
	{
		RC_Chassis_Speed_Buffer[10] =('k');//校验位
		send_check = 1;
	}
	else if(send_check == 1)
	{
		RC_Chassis_Speed_Buffer[10] =('j');//校验位
		send_check = 0;
	}
	if(Send_error()==1)
	{
		UART_IdleIT_init();
		RC_Chassis_Speed_Buffer[8] = ('p');
		RC_Chassis_Speed_Buffer[9] = ('p');
		RC_Chassis_Speed_Buffer[11] = ('p');
	}
	HAL_UART_Transmit_DMA(&TX_UART,RC_Chassis_Speed_Buffer,12);
}

void RC_Rescue_Move_Send(MOTOR_MOVE_t rescue_move)
{
//	uint8_t TX_Rescue[4];
	switch(rescue_move)
	{
		case out:
		{
			RC_Chassis_Speed_Buffer[8] =('c');
		}
		break;
		case in:
		{
			RC_Chassis_Speed_Buffer[8] =('C');
		}	
		break;
		case stop:
		{
			RC_Chassis_Speed_Buffer[8] =('p');
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
			RC_Chassis_Speed_Buffer[9] =('v');
		}
		break;
		case in:
		{
			RC_Chassis_Speed_Buffer[9] =('V');
		}	
		break;
		case stop:
		{
			RC_Chassis_Speed_Buffer[9] =('p');
		}	
		break;
		
		default: break;
	}
}

uint8_t Send_error(void)
{
	  if (RC_Chassis_Speed_Buffer[8] != 'c' || RC_Chassis_Speed_Buffer[8] != 'C' || RC_Chassis_Speed_Buffer[8] != 'p')
    {
        goto error;
    }
		
		if (RC_Chassis_Speed_Buffer[9] != 'v' || RC_Chassis_Speed_Buffer[9] != 'V' || RC_Chassis_Speed_Buffer[9] != 'p')
    {
        goto error;
    }
		
		return 0;
		
		error:
		return 1;
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
	HAL_UART_Receive_IT(&RX_UART,RX_Bufer,12);
}
uint8_t RX_temp[1]={1};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit_IT(&TX_UART,(uint8_t*)"1234",4);
	HAL_UART_Receive_IT(&RX_UART,RX_Bufer,12);
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


