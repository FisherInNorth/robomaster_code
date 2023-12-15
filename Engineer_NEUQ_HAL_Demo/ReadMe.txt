2023-12-3 5.999（修） tc、hpy、cq
修正了一些bug，新增键鼠模式遥控器无缝流转，优化夹爪翻转pid

2022-6-7 5.998 tc、hpy
tc：修复了键鼠模式下矿石翻转的bug
hpy：新增电推杆状态改变延时功能

2022-6-6 5.995 tc
修正了一小部分的矿石翻转的bug，但是依然存在前后矿石翻转丢包的情况
夹爪翻转新增有负载状态下的pid

2022-6-5 5.99 tc
增加了Clamp的.c文件

2022-6-5 5.99 cq
增加红外传感器

2022-6-5 5.99 hpy
写了串口的校验代码
把矿石翻转的问题调好了

2022-6-4 5.97 tc
合并了矿石翻转相关的代码

2022-6-4 5.97 cq
增加气动夹取代码

2022-5-30 5.96 hpy
电推杆改到了上板

2022-5-30 5.95 tc
调好了电推杆，夹爪旋转打磨

2022-5-30 5.94 hpy
添加了部分遥控器代码

2022-5-29 5.932 tc
改了串口发送，改了key的代码


2022-5-25 5.92 tc
修改了定时器，新增tim6；写了键盘控制底盘的代码；写了部分键盘控制取矿功能的代码；

/***********电推杆************/
定时器4用于PWM输出控制推杆
定时器4通道1对应电推杆3（右后），频率1kHz GPIOD, GPIO_PinSource12-->H-->IN3
定时器4通道2对应电推杆3（右后），频率1kHz GPIOD, GPIO_PinSource13-->G-->IN4
定时器4通道3对应电推杆4（左后），频率1kHz GPIOD, GPIO_PinSource14-->F-->IN1
定时器4通道4对应电推杆4（左后），频率1kHz GPIOD, GPIO_PinSource15-->E-->IN2

定时器5用于PWM输出控制推杆
定时器5通道1对应电推杆1（右前），频率1kHz GPIOH, GPIO_PinSource10-->D-->IN1
定时器5通道2对应电推杆1（右前），频率1kHz GPIOH, GPIO_PinSource11-->C-->IN2
定时器5通道3对应电推杆2（左前），频率1kHz GPIOH, GPIO_PinSource12-->B-->IN3
定时器5通道4对应电推杆2（左前），频率1kHz GPIOI, GPIO_PinSource0 -->A-->IN4
/************GPIO************/
#define CAN1_GPIO_CLK 								RCC_AHB1Periph_GPIOD
#define CAN1_RX_PIN                 	GPIO_Pin_0
#define CAN1_TX_PIN                 	GPIO_Pin_1
#define CAN1_RX_GPIO_PORT          		GPIOD
#define CAN1_TX_GPIO_PORT          		GPIOD
#define CAN1_RX_GPIO_CLK           		RCC_AHB1Periph_GPIOD
#define CAN1_TX_GPIO_CLK           		RCC_AHB1Periph_GPIOD
#define CAN1_AF_PORT                	GPIO_AF_CAN1
#define CAN1_RX_SOURCE              	GPIO_PinSource0
#define CAN1_TX_SOURCE              	GPIO_PinSource1 

#define CAN2_GPIO_CLK 								RCC_AHB1Periph_GPIOB
#define CAN2_RX_PIN                 	GPIO_Pin_12
#define CAN2_TX_PIN                 	GPIO_Pin_13
#define CAN2_RX_GPIO_PORT          		GPIOB
#define CAN2_TX_GPIO_PORT          		GPIOB
#define CAN2_RX_GPIO_CLK           		RCC_AHB1Periph_GPIOB
#define CAN2_TX_GPIO_CLK           		RCC_AHB1Periph_GPIOB
#define CAN2_AF_PORT                	GPIO_AF_CAN2
#define CAN2_RX_SOURCE              	GPIO_PinSource12
#define CAN2_TX_SOURCE              	GPIO_PinSource13 
