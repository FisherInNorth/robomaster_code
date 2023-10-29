#include "BSP_Can.h"
#include "BSP_Motor.h"
#include "BSP_Handle.h"
#include "BSP_Flip.h"
#include "BSP_Mineral.h"


CAN_RxHeaderTypeDef RX1message;

char RXdata[8];
	int i;
extern MOTOR_t mineral_F_motor;//前面的电机
extern MOTOR_t mineral_B_motor;//后面的电机
extern MOTOR_t mineral_R_motor;//右边的电机
extern MOTOR_t mineral_L_motor;//左边的电机
extern MOTOR_t flip_R_motor;
extern MOTOR_t flip_L_motor;
extern MOTOR_t handle_motor;


/*
can通信滤波器初始化
*/

uint8_t Bsp_CanInit(void)    
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	/*can1初始化*/
	
	canFilter.FilterBank=1;    																//筛选器组1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
	canFilter.SlaveStartFilterBank=14;												//can2筛选组起始编号
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//配置过滤器
	
	/*can2初始化*/
	canFilter.FilterBank=14;    															//筛选器组15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//配置过滤器
	
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 接收fifo 0不为空中断
	return status;

}

/**
   * @function函数：CAN1_Send_Mineral_Msg
   * @brief描述：CAN1发送机械臂电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */

void CAN1_Send_Mineral_Msg(uint8_t* msg)   
{		
	CAN_TxHeaderTypeDef TX1message;
	uint8_t TXdata[8];
	TX1message.StdId = 0x200;                        //电调标识符
	TX1message.IDE = CAN_ID_STD;
	TX1message.RTR = CAN_RTR_DATA;
	TX1message.DLC = 8;                              //发送长度 (x字节)
	TX1message.TransmitGlobalTime = DISABLE;
	
	for(uint16_t i=0;i<8;i++)
	TXdata[i] = msg[i];
	
	HAL_CAN_AddTxMessage(&hcan1, &TX1message, TXdata, CAN_FILTER_FIFO0);    //将数据储存进邮箱FIFOx
}


void CAN1_Send_Flip_Msg(uint8_t* msg)  
{		
	CAN_TxHeaderTypeDef TX1message;
	uint8_t TXdata[8];
	TX1message.StdId = 0x1FF;                        //电调标识符
	TX1message.IDE = CAN_ID_STD;
	TX1message.RTR = CAN_RTR_DATA;
	TX1message.DLC = 8;                              //发送长度 (x字节)
	TX1message.TransmitGlobalTime = DISABLE;
	
	for(uint16_t i=0;i<8;i++)
	TXdata[i] = msg[i];
	
	HAL_CAN_AddTxMessage(&hcan1, &TX1message, TXdata, CAN_FILTER_FIFO0);    //将数据储存进邮箱FIFOx
}

uint8_t CAN1_Send_Handle_Msg(uint8_t* msg)  
{	
	CAN_TxHeaderTypeDef TX1message;
	uint8_t TXdata[8];
	uint8_t status;
	TX1message.StdId = 0x1FF;                        //电调标识符
	TX1message.IDE = CAN_ID_STD;
	TX1message.RTR = CAN_RTR_DATA;
	TX1message.DLC = 8;                              //发送长度 (x字节)
	TX1message.TransmitGlobalTime = DISABLE;

	for(uint16_t i=0;i<8;i++)
	TXdata[i] = msg[i];
	
	status=HAL_CAN_AddTxMessage(&hcan1, &TX1message, TXdata, CAN_FILTER_FIFO0);    //将数据储存进邮箱FIFOx
	return status;
}

/**\\Engineer\../BSP/BSP_Handle.c\handle_motor.round_cnt
* @brief  电机can回调赋值函数
* @param MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current
* @retval void
* @attention
*/
void Record_Motor_Callback(MOTOR_t *motor, char *data)
{
	motor->last_angle = motor->apid.actual_angle;
	motor->apid.actual_angle = (int16_t)((data[0] << 8) + data[1]);
	motor->vpid.actual_speed = (int16_t)((data[2] << 8) + data[3]);
	motor->actual_current = (int16_t)((data[4] << 8) + data[5]);
	if(motor->start_angle_flag == 0)
	{
		motor->start_angle = (int16_t)((data[0] << 8) + data[1]);
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}

	if(motor->apid.actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->apid.actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;	// - motor->start_angle;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RX1message,(uint8_t *)RXdata);
    if (hcan == &hcan1){
        switch (RX1message.StdId){
        case CANRX_Mineral_Motor_F:
					Record_Motor_Callback(&mineral_F_motor,RXdata);    
					break;
				case CANRX_Mineral_Motor_B:						
					Record_Motor_Callback(&mineral_B_motor,RXdata);            									
					break;				
				case CANRX_Mineral_Motor_R:					
					Record_Motor_Callback(&mineral_R_motor,RXdata);           									
					break;
				case CANRX_Mineral_Motor_L:					
					Record_Motor_Callback(&mineral_L_motor,RXdata);
					break;			
        case CANRX_Flip_Motor_R:
					Record_Motor_Callback(&flip_R_motor,RXdata);
					break;
				case CANRX_Flip_Motor_L:
					Record_Motor_Callback(&flip_L_motor,RXdata);
					break;
				case CANRX_Handle_Motor:
					Record_Motor_Callback(&handle_motor,RXdata);
					break;

       }
    }
    if (hcan == &hcan2){
        switch (RX1message.StdId){
        }
	}
}
