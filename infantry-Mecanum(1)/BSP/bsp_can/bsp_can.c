#include "bsp_can.h"

static int16_t* read_motor_data(uint8_t *rxdata);
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);
static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current);
static void canTX_gimbal(int8_t robot_id, int16_t heat0, int16_t heat0_limit ,int16_t bullet_speed, uint8_t bullet_speedlimit);

/**
  * @breif         canͨ�ų�ʼ��
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
uint8_t bsp_can_init(void)
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	
	canFilter.FilterBank=1;    																//ɸѡ����1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//����ģʽ
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//����
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32λģʽ
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//���ӵ�fifo0
	canFilter.SlaveStartFilterBank=14;												//can2ɸѡ����ʼ���
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//���ù�����
	
	canFilter.FilterBank=15;    															//ɸѡ����15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//���ù�����
	
	/*�뿪��ʼģʽ*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*���ж�*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 ����fifo 0��Ϊ���ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 ����fifo 0��Ϊ���ж�
	return status;
}


/**
  * @breif         can���ͺ���
  * @param[in]     hcan��can�ľ���ṹ��
	* @param[in]     mdata����Ҫ���͵�����
	* @param[out]    none
  * @retval        status�����͵�״̬    
  */
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)
{
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	if(hcan==&hcan1)
	{
		CAN_TxHeaderStruct.StdId=0x200;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	else if(hcan==&hcan2)
	{
		CAN_TxHeaderStruct.StdId=0x006;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
	return status;
}

/**
  * @breif         can�����жϺ���
  * @param[in]     hcan��can�ľ���ṹ��
	* @param[out]    can1�����ĸ�������صĵ���ֵ��can2�����ϰ崫��������
  * @retval        none   
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
		gdata=read_motor_data(rxdata);
		angle=gdata[0];
		speed=gdata[1];
		current=gdata[2];
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
			case CAN_GIMBAL_Y_ID:
				record_chassis_callback(&chassis_center,angle,current);
			break;
			case LOOP_BACK_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			default:break;
		}
	}
	else if(hcan==&hcan2)
	{
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
			chassis_control_order.chassis_mode=CHASSIS_NO_FORCE;
		gdata=read_motor_data(rxdata);
		if(CAN_RxHeaderStruct.StdId==GIMBAL_CONTROL_ID_1)
		{
			chassis_data_analyze();
		}
		if(CAN_RxHeaderStruct.StdId==GIMBAL_CONTROL_ID_2)	
		{
			chassis_control_order.last_chassis_mode=chassis_control_order.chassis_mode; 
			chassis_control_order.chassis_mode=gdata[0];
			if((chassis_control_order.chassis_mode==CHASSIS_FOLLOW)&&(chassis_control_order.last_chassis_mode==CHASSIS_SPIN))     
				chassis_center.switch_mode_flag=SPIN_TO_FOLLOW; 
			if(chassis_control_order.chassis_mode==CHASSIS_SPIN)
			{
				if((chassis_control_order.gimbal_6020_angle>5)&&(chassis_control_order.gimbal_6020_angle<360))
				{
					if(chassis_control_order.gimbal_6020_angle-chassis_control_order.last_gimbal_6020_angle>0)
						chassis_center.spin_dirt=CLOCKWISE;
					else if(chassis_control_order.gimbal_6020_angle-chassis_control_order.last_gimbal_6020_angle<0)
						chassis_center.spin_dirt=ANTICLOCKWISE;
				}
			}			
		}
	}
	
}

/**
  * @breif         can���ϵ�����ݺ���
  * @param[in]     rxdata��������ص�����
	* @param[out]    ���Ϻõ�����
  * @retval        adata�����Ϻõ�����  
  */
int16_t adata[4];
static int16_t* read_motor_data(uint8_t *rxdata)
{
		adata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
		adata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
		adata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
		adata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	return adata;
}

/**
  * @breif         can����������ݺ���
  * @param[in]     motor����������ṹ��
  * @param[in]     angle������Ƕ�ֵ
  * @param[in]     speed������ٶ�ֵ
  * @param[in]     current���������ֵ
	* @param[out]    �������ֵ��ֵ������Ľṹ���Ա
  * @retval        none   
  */
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->pid.position_loop.apid.actual_angle=angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->pid.position_loop.vpid.actual_speed=motor->actual_speed;
	motor->pid.speed_loop.vpid.actual_speed=motor->actual_speed;
	//motor->actual_speed = KalmanFilter(speed,Q,R);
	motor->last_speed = speed;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}
	
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt--;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt++;
	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;// - motor->start_angle;
}

static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = (float)angle/8191.0f;
	motor->actual_angle*=360.0f;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = (float)angle/8191.0f*360.0f;
		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}
	
}

void send_gimbal_data(void)
{
	uint8_t id,bullet_freq,bullet_speedlimit;
	uint16_t heat0,heat0_limit;
	float bullet_speed;
	id=get_robot_id();
	get_shoot_heat0_limit_and_heat0(&heat0_limit,&heat0);
	get_shoot_data( &bullet_freq, &bullet_speed, &bullet_speedlimit);
	canTX_gimbal(id, heat0, heat0_limit ,(int16_t)bullet_speed, bullet_speedlimit);
	
}

/**
  * @breif         can�����ϰ庯��
  * @param[in]     robot_id������id��
  * @param[in]     heat0��ǹ������
  * @param[in]     heat0_limit��ǹ����������
  * @param[in]     bullet_speed���ӵ�����
  * @param[in]     bullet_speedlimit���ӵ���������
	* @param[out]    none
  * @retval        none   
  */
static void canTX_gimbal(int8_t robot_id, int16_t heat0, int16_t heat0_limit ,int16_t bullet_speed, uint8_t bullet_speedlimit)
{
	uint8_t data[8]={0};
	
	data[0]=robot_id;
	data[1]=heat0>>8;
	data[2]=heat0&0xff;
	data[3]=heat0_limit>>8;
	data[4]=heat0_limit&0xff;
	data[5]=bullet_speed>>8;
	data[6]=bullet_speed&0xff;
	data[7]=bullet_speedlimit;
	Can_Tx_Message(&hcan2,data);	
}
void send_gimbal_data_2(void)
{
	int16_t actual_vx,actual_vy;
	actual_vx=(int16_t)Kinematics.actual_velocities.linear_x;
	actual_vy=(int16_t)Kinematics.actual_velocities.linear_y;
	canTX_gimbal_2(actual_vx,actual_vy);
}

void canTX_gimbal_2(int16_t vx,int16_t vy)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CHASSIS_TO_GIMBAL_ID_2;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=vx>>8;
	data[1]=vx&0xff;
	data[2]=vy>>8;
	data[3]=vy&0xff;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}
