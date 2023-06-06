#include "DR16.h"
#include "dma.h"
#include "usart.h"
#include "bsp_led.h"
#include "UpperBoard.h"
#include "Timer_Task.h"

#define RC_huart    huart3
#define RC_UART		USART3
#define RC_dma		hdma_usart3_rx
//气泵
#define Pump1_ON 		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)
#define Pump1_OFF 		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
#define Pump2_ON 		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
#define Pump2_OFF 		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#define Pump1_Tog  	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13)
#define Pump2_Tog 	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11)

extern DMA_HandleTypeDef hdma_usart3_rx;

extern float Outboard_MotorL_aPID_Parameters[];
extern float Outboard_MotorR_aPID_Parameters[];

int Key_Mode = 0;
uint16_t last_key=0;
uint16_t time_count_v=0;
uint16_t KEY_COUNT=6;
uint8_t flag_v=0;

uint8_t calibrate_lift_flag=1;

RC_ctrl_t rc_ctrl = {{0x0400,0x0400,0x0400,0x0400,2,2,0x0400},{0,0,0,0,0,},{0}};

/*******
ch
0  右通道 左右
1  右通道 前后
2  左通道 左右
3  左通道 前后
4  左上
s
0   左
1   右
*/

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
static int16_t RC_abs(int16_t value);
static void RC_restart(uint16_t dma_buf_num);
static float caculate_linear_speed(int width,int mid,int min,int max);

static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//中间消除波动
    speed=(3.0*(width-(mid+2))/(max-(mid+2))*NORMAL_LINEAR_SPEED);
  else if(width<=(mid-2))
    speed=(3.0*(width-(mid-2))/((mid-2)-min)*NORMAL_LINEAR_SPEED);
  else
    speed=0;
  return speed;                
}

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//中间消除波动
    speed=(3.0f*(width-(mid+2))/(max-(mid+2))*NORMAL_ROTATIONAL_SPEED);
  else if(width<=(mid-2))
    speed=(3.0f*(width-(mid-2))/((mid-2)-min)*NORMAL_ROTATIONAL_SPEED);
  else
    speed=0;
  return speed*1.5f;                
}

//主函数初始化调用
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//enable the DMA transfer for the receiver request
	//使能 DMA 串口接收
	SET_BIT(RC_huart.Instance->CR3, USART_CR3_DMAR);
	//enalbe idle interrupt
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&RC_huart, UART_IT_IDLE);
	//disable DMA
	//失效 DMA
	__HAL_DMA_DISABLE(&RC_dma);

	while(RC_dma.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&RC_dma);
	}

	RC_dma.Instance->PAR = (uint32_t) & (RC_UART->DR);
	//memory buffer 1
	//内存缓冲区 1
	RC_dma.Instance->M0AR = (uint32_t)(rx1_buf);
	//memory buffer 2
	//内存缓冲区 2
	RC_dma.Instance->M1AR = (uint32_t)(rx2_buf);
	//data length
	//数据长度
	RC_dma.Instance->NDTR = dma_buf_num;
	//enable double memory buffer
	//使能双缓冲区
	SET_BIT(RC_dma.Instance->CR, DMA_SxCR_DBM);
	//enable DMA
	//使能 DMA
	__HAL_DMA_ENABLE(&RC_dma);
}

//外部调用
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > 1684)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > 1684)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > 1684)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > 1684)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = 2;
    rc_ctrl.rc.s[1] = 2;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

//写入stm32f4xx_it.c串口中断

void USART3_IRQHandler(void)
{
	LED_Blue(1);
	if(RC_huart.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
	}
	else if(RC_UART->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
		if ((RC_dma.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */
			//disable DMA
			//失效 DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//重新设定数据长度
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 1
			//设定缓冲区 1
			RC_dma.Instance->CR |= DMA_SxCR_CT;
			//enable DMA
			//使能 DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效 DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//重新设定数据长度
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 0
			//设定缓冲区 0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			//enable DMA
			//使能 DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
	}
}

static void RC_restart(uint16_t dma_buf_num)
{
	//disable UART
	__HAL_UART_DISABLE(&RC_huart);
	//disable DMA
	__HAL_DMA_DISABLE(&RC_dma);
	//reset set_data_lenght
	RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
	__HAL_UART_CLEAR_IDLEFLAG(&RC_huart);
	__HAL_DMA_CLEAR_FLAG(&RC_dma,DMA_FLAG_TCIF2_6);
	// DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	//ensable UART
	__HAL_UART_ENABLE(&RC_huart);
	//ensable DMA
	__HAL_DMA_ENABLE(&RC_dma);
}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
			return;
	}
	
	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;//!< Channel 2
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003); //!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch right
	
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis
	
	rc_ctrl->mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press 
	rc_ctrl->mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press 
	
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value
	
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); //NULL
	

}


//void HandleDR16_Data(void)
//{
//		
//	/*操作模式一：抬升以及底盘运动 @判定方式：左拨杆开关为上*/
//	if(rc_ctrl.rc.s[1] == 1)
//	{
//		if(rc_ctrl.rc.s[0] == 3)//右拨杆开关为中：停止抬升
//		{
//			Lift_Motor_State = 0;
//		}
//		else if(rc_ctrl.rc.s[0] == 1)//右拨杆开关为上：抬升
//		{
//			Lift_Motor_State = 3;
//		}
//		else if(rc_ctrl.rc.s[0] == 2)//右拨杆开关为下：下降
//		{
//			Lift_Motor_State = 4;
//		}
//		
//		chassis_vy=caculate_linear_speed(rc_ctrl.rc.ch[2],RC_MIDD,RC_MINN,RC_MAXX);
//		chassis_vx=caculate_linear_speed(rc_ctrl.rc.ch[3],RC_MIDD,RC_MINN,RC_MAXX);
//		chassis_wz=-caculate_rotational_speed(rc_ctrl.rc.ch[0],RC_MIDD,RC_MINN,RC_MAXX);
//	}
//	
//	/*操作模式二：前后伸缩、两轴机械臂 @判定方式：左拨杆开关为中*/
//	if(rc_ctrl.rc.s[1] == 3)
//	{
//		if(rc_ctrl.rc.s[0] == 3)//右拨杆开关为中：停止伸缩
//		{
//			Longitudinal_Motor_State = 0;
//		}
//		else if(rc_ctrl.rc.s[0] == 1)//右拨杆开关为上：伸出
//		{
//			Longitudinal_Motor_State = 3;
//		}
//		else if(rc_ctrl.rc.s[0] == 2)//右拨杆开关为下：收回
//		{
//			Longitudinal_Motor_State = 4;
//		}
//		
//		
//		//右摇杆上下，控制pitch轴
//		if(rc_ctrl.rc.ch[1] > 1374)
//		{
//			Chuck_Pitch_State = 3;
//		}
//		else if(rc_ctrl.rc.ch[1] < 674)
//		{
//			Chuck_Pitch_State = 4;
//		}
//		else 
//		{
//			Chuck_Pitch_State = 0;
//		}
//		//右摇杆左右，控制roll轴
//		if(rc_ctrl.rc.ch[0] > 1374)  //roll
//		{
//			Chuck_Roll_State = 3;
//		}
//		else if(rc_ctrl.rc.ch[0] < 674)
//		{
//			Chuck_Roll_State = 4;
//		}
//		else
//		{
//			Chuck_Roll_State = 0;
//		}
//		



//		if(rc_ctrl.rc.ch[3]<674)//左摇杆向下，矿石向后翻转
//		{
//			mineral_motor1.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor2.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor3.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor4.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//		}
//		else if(rc_ctrl.rc.ch[3]>1374)//左摇杆向上，矿石向前翻转
//		{
//			mineral_motor1.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor2.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor3.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor4.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//		}
//		else
//		{
//			mineral_motor1.vpid.target_speed = 0;
//			mineral_motor2.vpid.target_speed = 0;
//			mineral_motor3.vpid.target_speed = 0;
//			mineral_motor4.vpid.target_speed = 0;
//		}
//		//上下
//			
//		
//		
//	}
//	
//		
//	/*操作模式三：气泵 @判定方式：左拨杆开关为下*/
//	if(rc_ctrl.rc.s[1] == 2)
//	{
//		if (rc_ctrl.rc.s[0] == 1)
//		{
//			Pump1_ON;
//			Pump2_ON;
//		}
//		else
//		{
//			Pump1_OFF;
//			Pump2_OFF;
//		}
//		
//		if(rc_ctrl.rc.ch[3]>1374)//左摇杆向上，矿石向上抬升
//		{
//			mineral_motor1.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor2.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor3.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor4.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//		}
//		else if(rc_ctrl.rc.ch[3]<674)//左摇杆向下，矿石向下存储
//		{
//			mineral_motor1.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor2.vpid.target_speed = - (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor3.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//			mineral_motor4.vpid.target_speed = (rc_ctrl.rc.ch[3] - 1024) * 5;
//		}
//		else
//		{
//			mineral_motor1.vpid.target_speed = 0;
//			mineral_motor2.vpid.target_speed = 0;
//			mineral_motor3.vpid.target_speed = 0;
//			mineral_motor4.vpid.target_speed = 0;
//		}
//	}


//}

////void remote_control(void)
////{
////		chassis_control_order.vy_set=caculate_linear_speed(rc_ctrl.rc.ch[0],RC_MIDD,RC_MINN,RC_MAXX);
////		chassis_control_order.vx_set=caculate_linear_speed(rc_ctrl.rc.ch[1],RC_MIDD,RC_MINN,RC_MAXX);
////		chassis_control_order.wz_set=caculate_rotational_speed(rc_ctrl.rc.ch[2],RC_MIDD,RC_MINN,RC_MAXX);
////	
////	switch(rc_ctrl.rc.s[1])
////	{
////		case 3: {chassis_control_order.chassis_mode=CHASSIS_NO_FORCE;} break;
////		case 1: {chassis_control_order.chassis_mode=CHASSIS_FOLLOW;} break;
////		case 2: {chassis_control_order.chassis_mode=CHASSIS_SPIN;} break;
////		default:break;
////	}
////}

//void Key_Control(void)
//{
//	Lift_Motor_State = 0;
//	Longitudinal_Motor_State = 0;
//	Chuck_Pitch_State = 0;
//	Chuck_Roll_State = 0;
//	
//	
//	
//	if(abs(rc_ctrl.mouse.x)<500 && abs(rc_ctrl.mouse.x)>20)
//	{
//		chassis_wz=-caculate_rotational_speed(rc_ctrl.mouse.x,0,-500,500)*3;
//	}
//	else chassis_wz=0;
//	
//	//图传舵机抬升和下降
//	if(abs(rc_ctrl.mouse.y)<500 && abs(rc_ctrl.mouse.y)>20)
//	{
//		if(caculate_rotational_speed(rc_ctrl.mouse.y,0,-500,500)>0)
//			View_Servo_State=1;
//		else if(caculate_rotational_speed(rc_ctrl.mouse.y,0,-500,500)<0)
//			View_Servo_State=2;
//	}
//	else
//		View_Servo_State=0;
//	
//	
//	
//	
//	if(flag_v == 0)
//	{
//		if((rc_ctrl.key.v & KEY_W) && (!(rc_ctrl.key.v&KEY_S)))
//			chassis_vx=70;
//		else if((rc_ctrl.key.v & KEY_S) && (!(rc_ctrl.key.v & KEY_W)))
//			chassis_vx=-70;
//		else
//			chassis_vx=0;
//		
//		if((rc_ctrl.key.v & KEY_D) && (!(rc_ctrl.key.v&KEY_A)))
//			chassis_vy=70;
//		else if((rc_ctrl.key.v & KEY_A) && (!(rc_ctrl.key.v&KEY_D)))
//			chassis_vy=-70;
//		else
//			chassis_vy=0;
//		
//		if((rc_ctrl.key.v & KEY_Q) && (!(rc_ctrl.key.v & KEY_E)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Lift_Motor_State = 4;
//	//		outboard_lift_motorL.apid.target_angle +=2250;
//	//		outboard_lift_motorR.apid.target_angle -=2250;
//			outboard_lift_motorL.apid.target_angle +=3500;
//			outboard_lift_motorR.apid.target_angle -=3500;
//			Chuck_Pitch_State = 0;
//		}
//		else if((rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Lift_Motor_State = 3;
//			outboard_lift_motorL.apid.target_angle -=3500;
//			outboard_lift_motorR.apid.target_angle +=3500;
//			Chuck_Pitch_State = 0;
//		}
//		
//		if((rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_C)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Longitudinal_Motor_State = 4;
//			Chuck_Roll_State = 0;
//		}
//		else if((rc_ctrl.key.v & KEY_C) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Longitudinal_Motor_State = 3;
//			Chuck_Roll_State = 0;
//		}
// }
//	else
//	{
//		if((rc_ctrl.key.v & KEY_W) && (!(rc_ctrl.key.v&KEY_S)))
//			chassis_vx=20;
//		else if((rc_ctrl.key.v & KEY_S) && (!(rc_ctrl.key.v & KEY_W)))
//			chassis_vx=-20;
//		else
//			chassis_vx=0;
//		
//		if((rc_ctrl.key.v & KEY_D) && (!(rc_ctrl.key.v&KEY_A)))
//			chassis_vy=20;
//		else if((rc_ctrl.key.v & KEY_A) && (!(rc_ctrl.key.v&KEY_D)))
//			chassis_vy=-20;
//		else
//			chassis_vy=0;		
//		
//		if((rc_ctrl.key.v & KEY_Q) && (!(rc_ctrl.key.v & KEY_E)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Lift_Motor_State = 6;
//			Chuck_Pitch_State = 0;
//		}
//		else if((rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Lift_Motor_State = 7;
//			Chuck_Pitch_State = 0;
//		}
//		
//		if((rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_C)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Longitudinal_Motor_State = 6;
//			Chuck_Roll_State = 0;
//		}
//		else if((rc_ctrl.key.v & KEY_C) && (!(rc_ctrl.key.v & KEY_Z)) && (!(rc_ctrl.key.v & KEY_SHIFT)))
//		{
//			Longitudinal_Motor_State = 7;
//			Chuck_Roll_State = 0;
//		}
//	}
//	
//	//气泵
//	if(rc_ctrl.mouse.press_l)
//		Pump_State=1;
//	if(rc_ctrl.mouse.press_r)
//		Pump_State=2;
//	
//	if(Pump_State==1)
//	{
//		Pump1_ON;
//		Pump2_ON;
//	}
//	else if(Pump_State==2)
//	{
//		Pump1_OFF;
//		Pump2_OFF;
//	}
//	

//	if((rc_ctrl.key.v & KEY_Q) && (!(rc_ctrl.key.v & KEY_E)) && (rc_ctrl.key.v & KEY_SHIFT))
//	{
//		Lift_Motor_State = 0;
//		Chuck_Pitch_State = 4;
//	}
//	else if((rc_ctrl.key.v & KEY_E) && (!(rc_ctrl.key.v & KEY_Q)) && (rc_ctrl.key.v & KEY_SHIFT))
//	{
//		Lift_Motor_State = 0;
//		Chuck_Pitch_State = 3;
//	}
////	else if((rc_ctrl.key.v & KEY_V) && (!(rc_ctrl.key.v & KEY_B))&& (!(rc_ctrl.key.v & KEY_SHIFT)))
////	{
////		Lift_Motor_State = 6;
////		Chuck_Pitch_State =0;
////	}
////	else if((rc_ctrl.key.v & KEY_B) && (!(rc_ctrl.key.v & KEY_V))&& (!(rc_ctrl.key.v & KEY_SHIFT)))
////	{
////		Lift_Motor_State = 7;
////		Chuck_Pitch_State =0;
////	}	
//	else if((rc_ctrl.key.v & KEY_B) && (!(rc_ctrl.key.v & KEY_V))&& (!(rc_ctrl.key.v & KEY_SHIFT)))
//	{
//		Lift_Motor_State = 0;
//		Chuck_Pitch_State = 5;
//	}
//	
////	if(outboard_lift_motorL.apid.target_angle>0)
////	{
////		outboard_lift_motorL.apid.target_angle = 0;
////	}
////	if(outboard_lift_motorR.apid.target_angle<0)
////	{
////		outboard_lift_motorR.apid.target_angle = 0;
////	}
//	

//	else if((rc_ctrl.key.v & KEY_Z) && (!(rc_ctrl.key.v & KEY_C)) && (rc_ctrl.key.v & KEY_SHIFT))
//	{
//		Longitudinal_Motor_State = 0;
//		Chuck_Roll_State = 3;
//	}
//	else if((rc_ctrl.key.v & KEY_C) && (!(rc_ctrl.key.v & KEY_Z)) && (rc_ctrl.key.v & KEY_SHIFT))
//	{
//		Longitudinal_Motor_State = 0;
//		Chuck_Roll_State = 4;
//	}
//	else if(!(rc_ctrl.key.v & KEY_V) && (rc_ctrl.key.v & KEY_B)&& (rc_ctrl.key.v & KEY_SHIFT))
//	{
//		Chuck_Roll_State = 5;
//		Longitudinal_Motor_State =0;
//	}

//	
//	if(rc_ctrl.key.v & KEY_F)
//	{
//		Lift_Motor_State = 5;
//		if(Longitudinal_Ready_State == 0)
//		{
//			outboard_lift_motorL.apid.target_angle -=3500;
//			outboard_lift_motorR.apid.target_angle +=3500;
//		}
//		else if(Longitudinal_Ready_State == 1)
//		{
//			outboard_lift_motorL.apid.target_angle += 3500;
//			outboard_lift_motorR.apid.target_angle -= 3500;
//		}
//    
//	}

////	if(rc_ctrl.key.v & KEY_F)
////	{
////		Outboard_MotorL_aPID_Parameters[5] = 1700;
////		Outboard_MotorR_aPID_Parameters[5] = 1700;
////	}
////	else 
////	{
////		Outboard_MotorL_aPID_Parameters[5] = 1100;
////		Outboard_MotorR_aPID_Parameters[5] = 1100;
////	}
////		
//	
////if(rc_ctrl.key.v & KEY_B)
////{
//////		Longitudinal_Motor_State = 4;
//////		if(Longitudinal_Ready_State == 1)
//////		{
//////			Longitudinal_Motor_State = 0;
//////			outboard_lift_motorL.apid.target_angle -=2250;
//////			outboard_lift_motorR.apid.target_angle +=2250;
//////			
//////		}
////	Lift_Motor_State = 7;
////}
////else
////{
////	Lift_Motor_State = 0;
//////		Longitudinal_Motor_State = 0;
////}

////if(rc_ctrl.key.v & KEY_V)
////{
//////		Longitudinal_Motor_State = 4;
//////		if(Longitudinal_Ready_State == 1)
//////		{
//////			Longitudinal_Motor_State = 0;
//////			outboard_lift_motorL.apid.target_angle +=2250;
//////			outboard_lift_motorR.apid.target_angle -=2250;
//////			
//////		}
////	Lift_Motor_State = 6;
////}
////else
////{
////	Lift_Motor_State = 0;
//////		Longitudinal_Motor_State = 0;
////}
//	

//}

//void Key_Mode_Judge(void)
//{
//	if((rc_ctrl.rc.s[0] == 2) && (rc_ctrl.rc.s[1] == 2))
//	{
//		Key_Mode = 1;
//	}
//	else
//	{
//		Key_Mode = 0;
//	}
//}

void Motor_Lift_Calibrate()
{
	outboard_lift_motorL.apid.target_angle +=40;
	outboard_lift_motorR.apid.target_angle -=40;
  if((abs(outboard_lift_motorL.apid.target_angle - outboard_lift_motorL.apid.total_angle) > 40000) && (abs(outboard_lift_motorR.apid.target_angle - outboard_lift_motorR.apid.total_angle) > 40000))
	{
		if( (abs(outboard_lift_motorL.vpid.actual_speed) < 200) && (abs(outboard_lift_motorR.vpid.actual_speed) < 200) )
		{
			outboard_lift_motorL.apid.target_angle = outboard_lift_motorL.apid.total_angle;
			outboard_lift_motorR.apid.target_angle = outboard_lift_motorR.apid.total_angle;
			outboard_lift_motorL.round_cnt = 0;
			outboard_lift_motorR.round_cnt = 0;
			calibrate_lift_flag = 0;
		}
	}
}

void Motor_Control()
{
	if(Out_Lift_Motor_State == 3)
	{
			outboard_lift_motorL.apid.target_angle -=3500;
			outboard_lift_motorR.apid.target_angle +=3500;
	}
	else if(Out_Lift_Motor_State == 4)
	{
			outboard_lift_motorL.apid.target_angle +=3500;
			outboard_lift_motorR.apid.target_angle -=3500;
	}
	
	if(Chassis_vx_State == 3)
	{
		chassis_vx = 70;
	}
	else if(Chassis_vx_State == 4)
	{
		chassis_vx = -70;
	}
	else if(Chassis_vx_State == 6)
	{
		chassis_vx = 20;
	}
	else if(Chassis_vx_State == 7)
	{
		chassis_vx = -20;
	}
	else if(Chassis_vx_State == 0)
	{
		chassis_vx = 0;
	}
	
	if(Chassis_vy_State == 3)
	{
		chassis_vy = 70;
	}
	else if(Chassis_vy_State == 4)
	{
		chassis_vy = -70;
	}
	else if(Chassis_vy_State == 6)
	{
		chassis_vy = 20;
	}
	else if(Chassis_vy_State == 7)
	{
		chassis_vy = -20;
	}
	else if(Chassis_vy_State == 0)
	{
		chassis_vy = 0;
	}
	
	if(Chassis_wz_State == 3)
	{
		chassis_wz = -0.5;
	}
	else if(Chassis_wz_State == 4)
	{
		chassis_wz = 0.5;
	}
	else if(Chassis_wz_State == 0)
	{
		chassis_wz = 0;
	}
	
	if(Pump_State == 1)
	{
		Pump1_ON;
		Pump2_ON;
	}
	else
	{
		Pump1_OFF;
		Pump2_OFF;
	}
}

//	void judge_v(void)
//{
//	if(rc_ctrl.key.v&KEY_V)
//		time_count_v++;
//	else
//	{
//		if(last_key&KEY_V)
//		{
//			if(time_count_v>=KEY_COUNT)
//			{
//				if(flag_v==0)
//					flag_v=1;
//				else 
//					flag_v=0;
//			}
//		}
//		time_count_v=0;		
//	}
//	last_key = rc_ctrl.key.v;
//}