#include "bsp_can.h"


/*CAN1接收数据*/
unsigned char CAN_RxData[8];

/**
	* @function	函数：CAN_Filter_Config
	* @brief		描述：初始化CAN过滤器
	* @param		输入：无
	* @retval	返回值：无
	*/
void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef CAN_Filter;
	
	/*CAN1过滤器初始化*/
	CAN_Filter.FilterBank =	1;    													//筛选器组1[过滤器0-13为CAN1使用；过滤器14-28号为CAN2使用]
	CAN_Filter.FilterMode	=	CAN_FILTERMODE_IDMASK;  				//掩码模式[掩码模式/列表模式]
	CAN_Filter.FilterScale	= CAN_FILTERSCALE_32BIT; 				//过滤器尺度[32位/16位]
	CAN_Filter.FilterIdHigh = 0;														//32位下，存储要过滤ID的高16位
	CAN_Filter.FilterIdLow	=	0;														//32位下，存储要过滤ID的低16位
	CAN_Filter.FilterMaskIdHigh	=	0;												//掩码模式下，存储的是掩码(高16位)
	CAN_Filter.FilterMaskIdLow	=	0	;												//掩码模式下，存储的是掩码(低16位)
	CAN_Filter.FilterFIFOAssignment	=	CAN_FILTER_FIFO0; 		//报文通过过滤器的匹配后，存储到哪个FIFO
	CAN_Filter.FilterActivation	=	CAN_FILTER_ENABLE;				//使能过滤器
	CAN_Filter.SlaveStartFilterBank=14;										//？？？
	
	/*初始化CAN1过滤器*/
	HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter);				
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);
	/*使能CAN1的FIFO0不为空中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
	
	/*CAN2过滤器初始化*/
	CAN_Filter.FilterBank=14;    														//筛选器组15[过滤器0-13为CAN1使用；过滤器14-28号为CAN2使用]
	
	/*初始化CAN2过滤器*/
	HAL_CAN_ConfigFilter(&hcan2,&CAN_Filter);
	/*离开初始模式*/
	HAL_CAN_Start(&hcan2);
	/*使能CAN2的FIFO0不为空中断*/
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}




void CAN1_Send_Msg(uint32_t StdId, unsigned char * data)
{
	CAN_TxHeaderTypeDef CAN_TXHeader;
	uint32_t  pTxMailbox;
	
	/*指定标识符*/
	CAN_TXHeader.StdId = StdId;
	
	/*指定要传输的消息的标识符类型[标准标识符CAN_ID_STD,扩展标识符CAN_ID_EXT]*/
	CAN_TXHeader.IDE = CAN_ID_STD;
	
	/*指定要传输的消息的帧类型[数据帧CAN_RTR_DATA,遥控帧CAN_RTR_REMOTE]*/
	CAN_TXHeader.RTR = CAN_RTR_DATA;
	
	/*指定要传输的帧的长度*/
	CAN_TXHeader.DLC = 8;
	
	/*是否用时间戳取代第7、8位，若为True则帧的长度必须为8*/
	CAN_TXHeader.TransmitGlobalTime = DISABLE;
	
	/*将数据储存进邮箱FIFO0*/
	//HAL_CAN_AddTxMessage(&hcan1, &CAN_TXHeader, TXdata, CAN_FILTER_FIFO0);
	HAL_CAN_AddTxMessage(&hcan1, &CAN_TXHeader, data, &pTxMailbox);
}

void CAN2_Send_Msg(uint32_t StdId, unsigned char * data)
{
	CAN_TxHeaderTypeDef CAN_TXHeader;
	
	/*指定标识符*/
	CAN_TXHeader.StdId = StdId;
	
	/*指定要传输的消息的标识符类型[标准标识符CAN_ID_STD,扩展标识符CAN_ID_EXT]*/
	CAN_TXHeader.IDE = CAN_ID_STD;
	
	/*指定要传输的消息的帧类型[数据帧CAN_RTR_DATA,遥控帧CAN_RTR_REMOTE]*/
	CAN_TXHeader.RTR = CAN_RTR_DATA;
	
	/*指定要传输的帧的长度*/
	CAN_TXHeader.DLC = 8;
	
	/*是否用时间戳取代第7、8位，若为True则帧的长度必须为8*/
	CAN_TXHeader.TransmitGlobalTime = DISABLE;
	
	/*将数据储存进邮箱FIFO0*/
	HAL_CAN_AddTxMessage(&hcan2, &CAN_TXHeader, data, CAN_FILTER_FIFO0);
}


/**
	* @function	函数：HAL_CAN_RxFifo0MsgPendingCallback
	* @brief		描述：CAN接收回调函数
	* @param		输入：can结构体（该函数由HAL库调用，参数为hcan1，hcan2,......）
	* @retval	返回值：无
	*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeader;
	LED_Blue(1);
	/*读取数据*/
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData);
	if (hcan == &hcan1)
	{
		CAN1_Motor_HandleMsg(CAN_RxHeader.StdId, CAN_RxData);
	}
	else if (hcan == &hcan2)
	{
		CAN2_Motor_HandleMsg(CAN_RxHeader.StdId, CAN_RxData);
	}
}
