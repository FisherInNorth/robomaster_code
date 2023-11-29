#include "BSP_Usart.h"

static int16_t* read_motor_data(uint8_t *rxdata);

uint8_t rxdata[8];
int16_t *gdata;

/**
  * @breif         串口接受电机数据函数
  * @param[in]     void
	* @param[out]    void
  * @retval        
  */
void RX_chassis_speed()
{
	
    HAL_UART_Receive(&huart1,rxdata,8,0xff);
	  gdata=read_motor_data(rxdata);
		chassis_data_analyze();
	
}


/**
  * @breif         整合电机数据函数
  * @param[in]     rxdata：接受的电机速度的数组
	* @param[out]    整合好的数组
  * @retval        cdata：整合好的数组  
  */
int16_t cdata[4];
static int16_t* read_motor_data(uint8_t *rxdata)
{
		cdata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
		cdata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
		cdata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
	  cdata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	return cdata;
}
