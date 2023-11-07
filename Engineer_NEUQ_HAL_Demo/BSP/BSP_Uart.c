#include "BSP_Uart.h"
#include "Remote_Task.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t RC_CH_Buffer[9];
uint8_t RC_Chassis_Speed_Buffer[8];
uint8_t RX_Bufer[50];
uint32_t RecCount=0;
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
	
	HAL_UART_Transmit(&huart1,RC_CH_Buffer,9,0xff);
}

/**
   * @function函数：RC_Chassis_Speed_Send
   * @brief描述：发送底盘速度值
   * @param输入：x/y/r speed
   * @retval返回：void
   */
void RC_Chassis_Speed_Send(uint16_t speed1,uint16_t speed2,uint16_t speed3,uint16_t speed4)
{
	RC_Chassis_Speed_Buffer[0] =(speed1>>8);
	RC_Chassis_Speed_Buffer[1] =(speed1&0xff);
	RC_Chassis_Speed_Buffer[2] =(speed2>>8);
	RC_Chassis_Speed_Buffer[3] =(speed2&0xff);
	RC_Chassis_Speed_Buffer[4] =(speed3>>8);
	RC_Chassis_Speed_Buffer[5] =(speed3&0xff); 
	RC_Chassis_Speed_Buffer[6] =(speed4>>8);
	RC_Chassis_Speed_Buffer[7] =(speed4&0xff);
	
	HAL_UART_Transmit(&huart1,RC_Chassis_Speed_Buffer,8,0xff);
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
      HAL_UART_Transmit(&huart1,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}

//空闲中断初始化
void UART_IdleIT_init()
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,RX_Bufer,0xff);
}

void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
	RecCount=50-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	HAL_UART_Receive_DMA(&huart1,RX_Bufer,0xff);
}
	
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(__HAL_UART_GET_FLAG(&huart1,UART_IT_IDLE)==1)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		HAL_UART_IdleCpltCallback(&huart1);
		
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}


