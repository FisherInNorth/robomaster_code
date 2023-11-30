#ifndef __ANO_H
#define	__ANO_H

#include "struct_typedef.h"
#include "string.h"
#include "stdio.h"

/*��λ������ָ�������ֽڳ���*/
#define DEBUG_RX_BUF_NUM 12u
/***��������ͨ�Ÿ�ʽ ����ͷ0xAA Ӳ����ַ0xAF ��λ��****/
#define ANONY_HEAD	0xAA
#define ANONY_ADDR	0xAF
#define ANONY_ID1		0xF1	//�������֡ ID��Χ��0xF1~0xFA
#define ANONY_ID2		0xF2
#define ANONY_ID3		0xF3
#define ANONY_ID4		0xF4
#define ANONY_ID5		0xF5
#define ANONY_ID6		0xF6
#define ANONY_ID7		0xF7
#define ANONY_ID8		0xF8
#define ANONY_ID9		0xF9
#define ANONY_IDA		0xFA
#define ANONY_LEN		40		//�������ݶγ��� �40
#define STM32_ADDR	0xFF	//��λ�����յ�ַ
#define PC_ADDR			0xAF		//��λ�����յ�ַ
#define STM32_WRITE_CMD 0xE2	//��λ��д��,��������ID
#define STM32_READ_CMD	0xE1	//��λ����ȡID
#define TMS32_LOG_CMD		0xA0	//��λ����ʾ�ַ���
#define TMS32_LOGS_CMD		0xA1	//��λ����ʾ�ַ���+����
#define STM32_WRITE_LEN		6			//д��������
#define STM32_READ_LEN		6			//��ȡ������
#define RETURN_ID	0x00		//����֡ID
/**************************************
 *�пտ��Լ��ϲ�����ȡ�Ĵ���
 *����д�� dataweite	����д�뷵�� writereturn
 *������ȡ dataread		������ȡ���� readreturn
 *LOG��Ϣ���	�ַ���
 *LOG��Ϣ��� ����+�ַ���
**/
#pragma pack(push, 1) 
typedef struct 
{
	uint16_t PAR_ID;
	int32_t PAR_VAL;
}Debug_receive;

typedef struct 
{
	uint16_t DEBUG_ID;
	float DEBUG_VAL;
}Debug_data;
#pragma pack(pop)

extern void ANO_usart_init(void);
extern void DEBUG_strings(const char *pdata,uint8_t color);
extern void DEBUG_printf(const char *pdata,int32_t VAL);
extern void DEBUG_Anony16Send(uint8_t id,int16_t *pst,uint8_t len);
extern void DEBUG_Anony32Send(uint8_t id,int32_t *pst,uint8_t len);


#endif /**__ANO_H**/
