#include "bsp_can.h"


/*CAN1��������*/
unsigned char CAN_RxData[8];

/**
	* @function	������CAN_Filter_Config
	* @brief		��������ʼ��CAN������
	* @param		���룺��
	* @retval	����ֵ����
	*/
void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef CAN_Filter;
	
	/*CAN1��������ʼ��*/
	CAN_Filter.FilterBank =	1;    													//ɸѡ����1[������0-13ΪCAN1ʹ�ã�������14-28��ΪCAN2ʹ��]
	CAN_Filter.FilterMode	=	CAN_FILTERMODE_IDMASK;  				//����ģʽ[����ģʽ/�б�ģʽ]
	CAN_Filter.FilterScale	= CAN_FILTERSCALE_32BIT; 				//�������߶�[32λ/16λ]
	CAN_Filter.FilterIdHigh = 0;														//32λ�£��洢Ҫ����ID�ĸ�16λ
	CAN_Filter.FilterIdLow	=	0;														//32λ�£��洢Ҫ����ID�ĵ�16λ
	CAN_Filter.FilterMaskIdHigh	=	0;												//����ģʽ�£��洢��������(��16λ)
	CAN_Filter.FilterMaskIdLow	=	0	;												//����ģʽ�£��洢��������(��16λ)
	CAN_Filter.FilterFIFOAssignment	=	CAN_FILTER_FIFO0; 		//����ͨ����������ƥ��󣬴洢���ĸ�FIFO
	CAN_Filter.FilterActivation	=	CAN_FILTER_ENABLE;				//ʹ�ܹ�����
	CAN_Filter.SlaveStartFilterBank=14;										//������
	
	/*��ʼ��CAN1������*/
	HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter);				
	/*�뿪��ʼģʽ*/
	HAL_CAN_Start(&hcan1);
	/*ʹ��CAN1��FIFO0��Ϊ���ж�*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
	
	/*CAN2��������ʼ��*/
	CAN_Filter.FilterBank=14;    														//ɸѡ����15[������0-13ΪCAN1ʹ�ã�������14-28��ΪCAN2ʹ��]
	
	/*��ʼ��CAN2������*/
	HAL_CAN_ConfigFilter(&hcan2,&CAN_Filter);
	/*�뿪��ʼģʽ*/
	HAL_CAN_Start(&hcan2);
	/*ʹ��CAN2��FIFO0��Ϊ���ж�*/
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}




void CAN1_Send_Msg(uint32_t StdId, unsigned char * data)
{
	CAN_TxHeaderTypeDef CAN_TXHeader;
	uint32_t  pTxMailbox;
	
	/*ָ����ʶ��*/
	CAN_TXHeader.StdId = StdId;
	
	/*ָ��Ҫ�������Ϣ�ı�ʶ������[��׼��ʶ��CAN_ID_STD,��չ��ʶ��CAN_ID_EXT]*/
	CAN_TXHeader.IDE = CAN_ID_STD;
	
	/*ָ��Ҫ�������Ϣ��֡����[����֡CAN_RTR_DATA,ң��֡CAN_RTR_REMOTE]*/
	CAN_TXHeader.RTR = CAN_RTR_DATA;
	
	/*ָ��Ҫ�����֡�ĳ���*/
	CAN_TXHeader.DLC = 8;
	
	/*�Ƿ���ʱ���ȡ����7��8λ����ΪTrue��֡�ĳ��ȱ���Ϊ8*/
	CAN_TXHeader.TransmitGlobalTime = DISABLE;
	
	/*�����ݴ��������FIFO0*/
	//HAL_CAN_AddTxMessage(&hcan1, &CAN_TXHeader, TXdata, CAN_FILTER_FIFO0);
	HAL_CAN_AddTxMessage(&hcan1, &CAN_TXHeader, data, &pTxMailbox);
}

void CAN2_Send_Msg(uint32_t StdId, unsigned char * data)
{
	CAN_TxHeaderTypeDef CAN_TXHeader;
	
	/*ָ����ʶ��*/
	CAN_TXHeader.StdId = StdId;
	
	/*ָ��Ҫ�������Ϣ�ı�ʶ������[��׼��ʶ��CAN_ID_STD,��չ��ʶ��CAN_ID_EXT]*/
	CAN_TXHeader.IDE = CAN_ID_STD;
	
	/*ָ��Ҫ�������Ϣ��֡����[����֡CAN_RTR_DATA,ң��֡CAN_RTR_REMOTE]*/
	CAN_TXHeader.RTR = CAN_RTR_DATA;
	
	/*ָ��Ҫ�����֡�ĳ���*/
	CAN_TXHeader.DLC = 8;
	
	/*�Ƿ���ʱ���ȡ����7��8λ����ΪTrue��֡�ĳ��ȱ���Ϊ8*/
	CAN_TXHeader.TransmitGlobalTime = DISABLE;
	
	/*�����ݴ��������FIFO0*/
	HAL_CAN_AddTxMessage(&hcan2, &CAN_TXHeader, data, CAN_FILTER_FIFO0);
}


/**
	* @function	������HAL_CAN_RxFifo0MsgPendingCallback
	* @brief		������CAN���ջص�����
	* @param		���룺can�ṹ�壨�ú�����HAL����ã�����Ϊhcan1��hcan2,......��
	* @retval	����ֵ����
	*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeader;
	LED_Blue(1);
	/*��ȡ����*/
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
