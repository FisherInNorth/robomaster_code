#include "BSP_Can.h"

static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);

/**
  * @breif         can1通信初始化
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
void bsp_can1_init(void)
{
	CAN_FilterTypeDef canFilter;
	
	canFilter.FilterBank=0;    																//筛选器组1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
	canFilter.SlaveStartFilterBank=14;												//can2筛选组起始编号
	
	HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//配置过滤器
		
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);					
	
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
}

/**
  * @breif         can2通信初始化
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
void bsp_can2_init(void)
{
	CAN_FilterTypeDef canFilter;
	
	canFilter.FilterBank=14;    																//筛选器组14
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
	canFilter.SlaveStartFilterBank=14;												//can2筛选组起始编号
		
	HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//配置过滤器
	
	/*离开初始模式*/
	HAL_CAN_Start(&hcan2);
	
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 接收fifo 0不为空中断
}

/**
* @breif         can1底盘发送函数   ID:1-4
	* @param[in]     mdata：需要发送的数组
	* @param[out]    void 
  * @retval        void   
  */
void CAN1_Chassis_Msg(uint8_t *mdata)
{
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
		CAN_TxHeaderStruct.StdId=0x200;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
}

/**
* @breif         can1救援爪发送函数
	* @param[in]     mdata：需要发送的数组
	* @param[out]    void 
  * @retval        void 
  */
void CAN1_Claw_Msg(uint8_t *mdata)
{
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
		CAN_TxHeaderStruct.StdId=0x1FF;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
}

/**
* @breif         can1复活卡发送函数
	* @param[in]     mdata：需要发送的数组
	* @param[out]    void 
  * @retval        void 
  */
void CAN1_Card_Msg(uint8_t *mdata)
{
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
		CAN_TxHeaderStruct.StdId=0x1FF;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
}

/**
  * @breif         can接收中断函数
  * @param[in]     hcan：can的句柄结构体
	* @param[out]    can1接收四个电机返回的电流值，can2接收上板传来的数据
  * @retval        none   
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t RXdata[8];
	int16_t speed,current;
	float angle;
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,RXdata);
		angle=(RXdata[0]<<8)|RXdata[1];
		speed=(RXdata[2]<<8)|RXdata[3];
		current=(RXdata[4]<<8)|RXdata[5];
		switch(CAN_RxHeaderStruct.StdId)
		{
			case CAN_3508Motor1_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			break;
			case CAN_3508Motor2_ID:
				get_motor_data(&chassis_motor2,angle,speed,current);
			break;
			case CAN_3508Motor3_ID:
				get_motor_data(&chassis_motor3,angle,speed,current);
			break;
			case CAN_3508Motor4_ID:
				get_motor_data(&chassis_motor4,angle,speed,current);
			break;
			case CAN_3508Motor5_ID:
				get_motor_data(&claw_L_motor,angle,speed,current);
			break;
			case CAN_3508Motor6_ID:
				get_motor_data(&claw_R_motor,angle,speed,current);
			break;
			case CAN_2006Motor7_ID:
				get_motor_data(&card_motor,angle,speed,current);
      break;			
			case LOOP_BACK_ID:           //不知道怎么用
				get_motor_data(&chassis_motor1,angle,speed,current);
			default:break;
		}
	}
	else if(hcan==&hcan2)
	{
	
	}
	
}


/**
  * @breif         can解析电机数据函数
  * @param[in]     motor：电机参数结构体
  * @param[in]     angle：电机角度值
  * @param[in]     speed：电机速度值
  * @param[in]     current：电机电流值
	* @param[out]    将输入的值赋值给电机的结构体成员
  * @retval        none   
  */
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->apid.actual_angle;
	motor->apid.actual_angle = angle;
	motor->vpid.actual_speed = speed;
	motor->actual_current = current;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
	
	if(motor->apid.actual_angle - motor->last_angle > 4096)
		motor->round_cnt--;
	else if (motor->apid.actual_angle - motor->last_angle < -4096)
		motor->round_cnt++;
	motor->apid.total_angle = motor->round_cnt * 8192 + motor->apid.actual_angle;// - motor->start_angle;
}



