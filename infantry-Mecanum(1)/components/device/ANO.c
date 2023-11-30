/**
  ******************************************************************************
  * @file    ANO.c
  * @author  ZWT
  * @version V1.0
  * @date    2021-xx-xx
  * @brief   ��������ʹ��
	*					 ʹ��USART ����V7ͨѶЭ�� ��д����
	*					 ���ڿ����жϽ��� DMA ˫����
	*					 ����DMA����
  ******************************************************************************
  * @attention
  * ƽ̨:		RoboMaster C�Ϳ����� STM32F407IGHx
  * QQ��		2457513659
  * Email:	zwt190315@163.com
  *	@reference 
	*		DJI C�Ϳ���������
	*		����V7ͨѶЭ��
  * @function
	*		void DEBUG_strings(const char *pdata,uint8_t color)
	*		void DEBUG_printf(const char *pdata,int32_t PAR_VAL)
  *		void DEBUG_Anony16Send(uint8_t id,int16_t *pst,uint8_t len)
	*		void DEBUG_Anony32Send(uint8_t id,int32_t *pst,uint8_t len)
  * @parament
  *		
  *	
  ******************************************************************************
**/ 
#include "main.h"
#include "ANO.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t ANO_rx_buffer[2][DEBUG_RX_BUF_NUM]={0};
static Debug_receive Debug_data_pack;//����д�����ݽṹ��

int32_t test[3]={0};
/****************�����������ڳ�ʼ��*******************************************/
/*
 *ʹ��USART1��DMA���պͷ���
 */
void ANO_usart_init()
{
	debug_usart_init(ANO_rx_buffer[0],ANO_rx_buffer[1],DEBUG_RX_BUF_NUM);
}
/****************�����������ͷ�����Ϣ*******************************************/
/**
  * @brief         	ANO���������ַ�����Ϣ
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *�����ַ��� brg��ʾ��ɫ �������255�ַ�
 *b=black=0;	r=rad=1;	g=green=2;
 */

void DEBUG_strings(const char *pdata,uint8_t color)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[256]={0};//�����255�ֽ�
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = TMS32_LOG_CMD;
	data_to_send[count++] = 0;//���ݳ�����ʱΪ0
	data_to_send[count++] = color;
	for (i=0;pdata[i]!='\0';i++)
	{
		data_to_send[count++] = pdata[i];
	}
	//data_to_send[count++] = 0;	����Ҫ���ַ��������ַ�'\0'
	data_to_send[3] = i+1;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	/*DMA��������*/
	debug_dma_tx(data_to_send,count);
}
/**
  * @brief         	ANO���������ַ���+������Ϣ
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *�����ַ���+����
 */
void DEBUG_printf(const char *pdata,int32_t PAR_VAL)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[256]={0};//�����255�ֽ�
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = TMS32_LOGS_CMD;
	data_to_send[count++] = 0;//���ݳ�����ʱΪ0
	memcpy(&data_to_send[4],&PAR_VAL,4);
	count+=4;
	for (i=0;pdata[i]!='\0';i++)
	{
		data_to_send[count++] = pdata[i];
	}
	//data_to_send[count++] = 0; //����Ҫ���ַ��������ַ�'\0'
	data_to_send[3] = i+4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	/*DMA��������*/
	debug_dma_tx(data_to_send,count);
}

/****************����������������*******************************************/
/**
	* @brief         	ANO�����ĵ������ݷ���
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *����2�ֽ����� int16�� -32767~+32767
 */
void DEBUG_Anony16Send(uint8_t id,int16_t *pst,uint8_t len)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[46];//�����40�ֽ� int16����20��
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = ANONY_ADDR;
	data_to_send[count++] = id;
	data_to_send[count++] = 0;
	len = len<10 ? len:10; 
	for (i=0;i<len;i++) //0~9 ʮ������
	{
		data_to_send[count++] = (unsigned char)pst[i]; 
		data_to_send[count++] = (unsigned char)(pst[i]>>8);  //��8λ
	}//С��ģʽ ���ֽ���ǰ ���ֽ��ں� һ�����10������
	data_to_send[3] = count - 4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	/*DMA��������*/
	debug_dma_tx(data_to_send,count);
	//HAL_UART_Transmit(&huart1,senddatas,20,1000);
}
/**
  * @brief         	ANO�����ĵ������ݷ���
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *����4�ֽ����� int32�� �ǳ��� ���Գ�����Ӧ10n�η���ת��ΪС��
 */
void DEBUG_Anony32Send(uint8_t id,int32_t *pst,uint8_t len)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[66];//�����60�ֽ� int16����40��
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = ANONY_ADDR;
	data_to_send[count++] = id;
	data_to_send[count++] = 0;
	len = len<10 ? len:10; 
	for (i=0;i<len;i++)
	{
		data_to_send[count++] = (unsigned char)pst[i]; 			//��8λ
		data_to_send[count++] = (unsigned char)(pst[i]>>8);  //��8λ
		data_to_send[count++] = (unsigned char)(pst[i]>>16); //��16λ
		data_to_send[count++] = (unsigned char)(pst[i]>>24); //��24λ
	}//С��ģʽ ���ֽ���ǰ ���ֽ��ں� һ�����10������
	data_to_send[3] = count - 4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	debug_dma_tx(data_to_send,count);
}
/**
  * @brief         	ANO�����Ĳ�����ȡ����
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
static void DEBUG_AnonyWriteReturn(uint8_t *pst)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[9];//��������֡��ʽ
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = RETURN_ID;
	data_to_send[count++] = 3;
	data_to_send[count++] = pst[0];
	data_to_send[count++] = pst[1];
	data_to_send[count++] = pst[2];
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	debug_dma_tx(data_to_send,count);
}
/**
  * @brief         	ANO�����Ĳ�����ȡ����
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
//int32_t LookVAL0=0,LookVAL1=0,LookVAL2=0;
//uint8_t LookArrys[6]={0};
static void DEBUG_AnonyReadReturn(uint16_t PAR_ID,int32_t PAR_VAL)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[12];//��������֡��ʽ
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = PC_ADDR;
	data_to_send[count++] = STM32_WRITE_CMD;
	data_to_send[count++] = 6;
	memcpy(&data_to_send[4],&PAR_ID,2);
	memcpy(&data_to_send[6],&PAR_VAL,4);
	count+=6;
//	data_to_send[count++] = (unsigned char)(PAR_ID&0x00FF);
//	data_to_send[count++] = (unsigned char)(PAR_ID&0xFF00)>>8;
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0x000000FF);
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0x0000FF00)>>8;
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0x00FF0000)>>16;
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0xFF000000)>>24;
//	LookVAL0=PAR_VAL;
//	memcpy(&LookArrys,&data_to_send[4],6);
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	debug_dma_tx(data_to_send,count);
}

/*****************************����������������*****************************/
 /**
  * @brief  Debug���ڵ����ݽ��պ���
  * @param  �����ֽ�
  * @retval ��
	* @attation
	* @note USART1 �����жϺ����е���
  */

void DEBUG_Data_solve(uint8_t *pData)
{
	uint8_t sumcheck = 0,addcheck = 0,i; 
	uint8_t return_data[3];
	uint16_t PAR_ID;
	int32_t PAR_VAL;
	if (pData[0]!=ANONY_HEAD) //����ͷ���ԣ������¿�ʼѰ��0xAA����ͷ
		return;
	switch (pData[2])
	{
		/*����д��ָ�� ���ض�ӦID������*/
		case STM32_WRITE_CMD:
			{
					for (i=0;i<pData[3]+4;i++)
				{
					sumcheck += pData[i];
					addcheck += sumcheck;
				}
				if(sumcheck == pData[10]&&addcheck == pData[11])
				{
					memcpy(&Debug_data_pack,&pData[4],6);	
					return_data[1] = pData[10];
					return_data[2] = pData[11];
					return_data[0] = STM32_WRITE_CMD;
					DEBUG_AnonyWriteReturn(return_data);
				}
				/*��ȡ����ID */
				switch (Debug_data_pack.PAR_ID)
				{
					case 0:test[0]=Debug_data_pack.PAR_VAL;break;
					case 1:test[1]=Debug_data_pack.PAR_VAL;break;
					case 2:test[2]=Debug_data_pack.PAR_VAL;break;
					default:break;
				}
			};break;
		/*���ݶ�ȡָ�� �޸Ķ�ӦID������*/
		case STM32_READ_CMD:
		{
					for (i=0;i<pData[3]+4;i++)
				{
					sumcheck += pData[i];
					addcheck += sumcheck;
				}
				if(sumcheck == pData[6]&&addcheck == pData[7])
				{
					memcpy(&PAR_ID,&pData[4],2);	
					switch (PAR_ID)
					{
						case 0:PAR_VAL=(int32_t)test[0];break;
						case 1:PAR_VAL=(int32_t)test[1];break;
						case 2:PAR_VAL=(int32_t)test[2];break;
						case 1000:PAR_VAL=0x11223344;break;
						default:break;
					}
					DEBUG_AnonyReadReturn(PAR_ID,PAR_VAL);
				}
		};break;
		default:break;
	}
}

/**
  * @brief         	ANO�����Ĵ����жϺ��� ������յ�����
  * @param[in]      none
  * @retval         none
	*	@attention			�����ж�
  */

void USART1_IRQHandler(void)
{
    if(USART1->SR & UART_FLAG_IDLE)
    {
			//static uint16_t this_time_rx_len=0;
			__HAL_UART_CLEAR_PEFLAG(&huart1);

			if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
			{
				/* Current memory buffer used is Memory 0 */

				//disable DMA
				//ʧЧDMA
				__HAL_DMA_DISABLE(&hdma_usart1_rx);

				//get receive data length, length = set_data_length - remain_length
				//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
				//this_time_rx_len = DEBUG_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
				
				
				//reset set_data_lenght
				//�����趨���ݳ���
				hdma_usart1_rx.Instance->NDTR = DEBUG_RX_BUF_NUM;

				//set memory buffer 1
				//�趨������1
				hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
				
				
				//enable DMA
				//ʹ��DMA
				__HAL_DMA_ENABLE(&hdma_usart1_rx);

				//������������
				if (ANO_rx_buffer[0][0]==ANONY_HEAD)
					DEBUG_Data_solve(ANO_rx_buffer[0]);
				memset(ANO_rx_buffer[0],0,DEBUG_RX_BUF_NUM);
				if (ANO_rx_buffer[1][0]==ANONY_HEAD)
					 DEBUG_Data_solve(ANO_rx_buffer[1]);

			 }
			else
			{
				/* Current memory buffer used is Memory 1 */
				//disable DMA
				//ʧЧDMA
				__HAL_DMA_DISABLE(&hdma_usart1_rx);

				//get receive data length, length = set_data_length - remain_length
				//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
				//this_time_rx_len = DEBUG_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

				//reset set_data_lenght
				//�����趨���ݳ���
				hdma_usart1_rx.Instance->NDTR = DEBUG_RX_BUF_NUM;

				
				//set memory buffer 0
				//�趨������0
				hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
				
				//enable DMA
				//ʹ��DMA
				__HAL_DMA_ENABLE(&hdma_usart1_rx);
			
				 //������������
				 if (ANO_rx_buffer[1][0]==ANONY_HEAD)
					DEBUG_Data_solve(ANO_rx_buffer[1]);
				 memset(ANO_rx_buffer[1],0,DEBUG_RX_BUF_NUM);
				 if (ANO_rx_buffer[0][0]==ANONY_HEAD)
					 DEBUG_Data_solve(ANO_rx_buffer[0]);
			 }
    }
}
/**************************************************************************/
