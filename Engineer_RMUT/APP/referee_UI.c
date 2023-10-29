#include "referee_UI.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "referee.h"
#include "bsp_referee.h"


ext_student_interactive_header_data_graphic_t ext_student_interactive_header_data_graphic;
ext_student_interactive_header_data_character_t ext_student_interactive_header_data_character;

uint8_t seq=0;

static void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	unsigned char i=i;
	
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(tx_buff,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	tx_buff[0] = sof;//数据帧起始字节
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	tx_buff[3] = seq;//包序号
	append_CRC8_check_sum(tx_buff,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	append_CRC16_check_sum(tx_buff,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****数据上传*****/
	__HAL_UART_CLEAR_FLAG(&huart8,UART_FLAG_TC);
	HAL_UART_Transmit(&huart8, tx_buff,frame_length , 100);
	while (__HAL_UART_GET_FLAG(&huart8,UART_FLAG_TC) == RESET); //等待之前的字符发送完成
}


static void get_UI_id(uint16_t *sender_ID,uint16_t *receiver_ID)
{
	switch(get_robot_id())
	{
		case UI_Data_RobotID_RHero:
		{
			*sender_ID=UI_Data_RobotID_RHero;
			*receiver_ID=UI_Data_CilentID_RHero;
			break;
		}
		case UI_Data_RobotID_REngineer:
		{
			*sender_ID=UI_Data_RobotID_REngineer;
			*receiver_ID=UI_Data_CilentID_REngineer;
			break;
		}
		case UI_Data_RobotID_RStandard1:
		{
			*sender_ID=UI_Data_RobotID_RStandard1;
			*receiver_ID=UI_Data_CilentID_RStandard1;
			break;
		}
		case UI_Data_RobotID_RStandard2:
		{
			*sender_ID=UI_Data_RobotID_RStandard2;
			*receiver_ID=UI_Data_CilentID_RStandard2;
			break;
		}
		case UI_Data_RobotID_RStandard3:
		{
			*sender_ID=UI_Data_RobotID_RStandard3;
			*receiver_ID=UI_Data_CilentID_RStandard3;
			break;
		}
		case UI_Data_RobotID_RAerial:
		{
			*sender_ID=UI_Data_RobotID_RAerial;
			*receiver_ID=UI_Data_CilentID_RAerial;
			break;
		}
		case UI_Data_RobotID_BHero:
		{
			*sender_ID=UI_Data_RobotID_BHero;
			*receiver_ID=UI_Data_CilentID_BHero;
			break;
		}
		case UI_Data_RobotID_BEngineer:
		{
			*sender_ID=UI_Data_RobotID_BEngineer;
			*receiver_ID=UI_Data_CilentID_BEngineer;
			break;
		}
		case UI_Data_RobotID_BStandard1:
		{
			*sender_ID=UI_Data_RobotID_BStandard1;
			*receiver_ID=UI_Data_CilentID_BStandard1;
			break;
		}	
		case UI_Data_RobotID_BStandard2:
		{
			*sender_ID=UI_Data_RobotID_BStandard2;
			*receiver_ID=UI_Data_CilentID_BStandard2;
			break;
		}	
		case UI_Data_RobotID_BStandard3:
		{
			*sender_ID=UI_Data_RobotID_BStandard3;
			*receiver_ID=UI_Data_CilentID_BStandard3;
			break;
		}	
		case UI_Data_RobotID_BAerial:
		{
			*sender_ID=UI_Data_RobotID_BAerial;
			*receiver_ID=UI_Data_CilentID_BAerial;
			break;
		}	
	}
}

uint16_t Sender_ID,Receiver_ID;
/************************************************绘制直线*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
void UI_draw_Line(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
	ext_student_interactive_header_data_graphic.data_cmd_id=0x0101;//绘制七个图形（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_graphic.sender_ID=Sender_ID;//发送者ID，机器人对应ID
	ext_student_interactive_header_data_graphic.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID
	//自定义图像数据
	
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye=UI_Graph_Line;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_y=Start_y;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_x=End_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_y=End_y;
	
	
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_graphic, sizeof(ext_student_interactive_header_data_graphic));	
}

/************************************************绘制矩形*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void UI_draw_Rectangle(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
	ext_student_interactive_header_data_graphic.data_cmd_id=0x0101;//绘制七个图形（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_graphic.sender_ID=Sender_ID;//发送者ID，机器人对应ID
	ext_student_interactive_header_data_graphic.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID
	//自定义图像数据
	
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye=UI_Graph_Rectangle;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_y=Start_y;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_x=End_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_y=End_y;
	
	
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_graphic, sizeof(ext_student_interactive_header_data_graphic));	
}
/************************************************绘制整圆*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_y    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
void UI_draw_Circle(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius)
{
	ext_student_interactive_header_data_graphic.data_cmd_id=0x0101;//绘制七个图形（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_graphic.sender_ID=Sender_ID;//发送者ID，机器人对应ID
	ext_student_interactive_header_data_graphic.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID
	//自定义图像数据
	
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye=UI_Graph_Circle;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_y=Start_y;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.radius=Graph_Radius;
	
	
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_graphic, sizeof(ext_student_interactive_header_data_graphic));	
}
/************************************************绘制圆弧*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
void UI_draw_Arc(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length)
{
	ext_student_interactive_header_data_graphic.data_cmd_id=0x0101;//绘制七个图形（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_graphic.sender_ID=Sender_ID;//发送者ID，机器人对应ID
	ext_student_interactive_header_data_graphic.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID
	//自定义图像数据
	
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye=UI_Graph_Arc;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_angle=Graph_StartAngle;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_angle=Graph_EndAngle;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_y=Start_y;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_x=x_Length;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_y=y_Length;
	
	
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_graphic, sizeof(ext_student_interactive_header_data_graphic));	
}  
/************************************************绘制浮点型数据*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_y    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
void UI_draw_Float(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float Graph_Float)
{
	ext_student_interactive_header_data_graphic.data_cmd_id=0x0101;//绘制一个图形（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_graphic.sender_ID=Sender_ID;//发送者ID，机器人对应ID
	ext_student_interactive_header_data_graphic.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID
	//自定义图像数据
	
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye=UI_Graph_Float;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_angle=Graph_Size;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_angle=Graph_Digit;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_y=Start_y;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.radius=(int32_t)(Graph_Float*1000);
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_x=(int32_t)(Graph_Float*1000)>>10;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_y=(int32_t)(Graph_Float*1000)>>21;
	
	
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_graphic, sizeof(ext_student_interactive_header_data_graphic));	
}  
/************************************************绘制整型数据*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Start_x、Start_y    开始坐标
        Graph_Int   要显示的变量
**********************************************************************************************************/
void UI_draw_Int(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,int32_t Graph_Int)
{
	ext_student_interactive_header_data_graphic.data_cmd_id=0x0101;//绘制一个图形（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_graphic.sender_ID=Sender_ID;//发送者ID，机器人对应ID
	ext_student_interactive_header_data_graphic.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID
	//自定义图像数据
	
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye=UI_Graph_Int;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_angle=Graph_Size;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.start_y=Start_y;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.radius=Graph_Int;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_x=Graph_Int>>10;
	ext_student_interactive_header_data_graphic.ext_client_custom_graphic_single.grapic_data_struct.end_y=Graph_Int>>21;
	
	
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_graphic, sizeof(ext_student_interactive_header_data_graphic));	
}  


/************************************************绘制字符型数据*************************************************
**参数：
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
void UI_character_draw_data(char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data)
{
	uint8_t i;
	ext_student_interactive_header_data_character.data_cmd_id=0x0110;//绘制字符（内容ID，查询裁判系统手册）
	get_UI_id(&Sender_ID,&Receiver_ID);
	ext_student_interactive_header_data_character.sender_ID=Sender_ID;//发送者ID，机器人对应ID，此处为蓝方英雄
	ext_student_interactive_header_data_character.receiver_ID=Receiver_ID;//接收者ID，操作手客户端ID，此处为蓝方英雄操作手客户端
	//自定义图像数据
	
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.graphic_name[0] = imagename[0];
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.graphic_name[1] = imagename[1];
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.graphic_name[2] = imagename[2];//图形名
	//上面三个字节代表的是图形名，用于图形索引，可自行定义
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.operate_tpye=Graph_Operate;//图形操作，0：空操作；1：增加；2：修改；3：删除；
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.graphic_tpye=UI_Graph_Char;//图形类型，0为直线，其他的查看用户手册
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.layer=Graph_Layer;//图层数
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.color=Graph_Color;//颜色
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.start_angle=Graph_Size;
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.end_angle=Graph_Digit;
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.width=Graph_Width;
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.start_x=Start_x;
	ext_student_interactive_header_data_character.ext_client_custom_character.grapic_data_struct.start_y=Start_y;
		
	for(i=0;i<Graph_Digit;i++)
	{	
		ext_student_interactive_header_data_character.ext_client_custom_character.data[i]=*Char_Data;
		Char_Data++;
	}
	referee_data_pack_handle(0xA5,0x0301, (uint8_t *)&ext_student_interactive_header_data_character, sizeof(ext_student_interactive_header_data_character));
}
//此处修改
extern uint8_t AirMineral_Mode;
extern uint8_t Keyboard_Mode;

static void UI_mode_send(uint32_t Graph_Operate)
{
	static char UI_char[30];
	if(Graph_Operate==UI_Graph_ADD)
	{
		if(Keyboard_Mode==ON)//修改
			sprintf((char *)UI_char,"Keyboard_Mode:ON");
		else
			sprintf((char *)UI_char,"Keyboard_Mode:OFF");		
		UI_character_draw_data("001",UI_Graph_ADD,1,UI_Color_Green,30,strlen(UI_char),3,50,SCREEN_WIDTH/3,UI_char);

		if(AirMineral_Mode==ON)//修改
			sprintf((char *)UI_char,"AirMineral_Mode:ON");
		else
			sprintf((char *)UI_char,"AirMineral_Mode:OFF");
		UI_character_draw_data("002",UI_Graph_ADD,1,UI_Color_Green,30,strlen(UI_char),3,50,SCREEN_WIDTH/3+50,UI_char);
	}
	else 
	{
		if(Keyboard_Mode==ON)//修改
			sprintf((char *)UI_char,"Keyboard_Mode:ON");
		else
			sprintf((char *)UI_char,"Keyboard_Mode:OFF");		
		UI_character_draw_data("001",UI_Graph_Change,1,UI_Color_Green,30,strlen(UI_char),3,50,SCREEN_WIDTH/3,UI_char);

		if(AirMineral_Mode==ON)//修改
			sprintf((char *)UI_char,"AirMineral_Mode:ON");
		else
			sprintf((char *)UI_char,"AirMineral_Mode:OFF");
		UI_character_draw_data("002",UI_Graph_Change,1,UI_Color_Green,30,strlen(UI_char),3,50,SCREEN_WIDTH/3+50,UI_char);
	}
}
void UI_Init(void)//放在初始化中
{
	UI_mode_send(UI_Graph_ADD);
}

void UI_Display(void)//放在定时器中断中，10hz
{
	UI_mode_send(UI_Graph_Change);
}



