#ifndef __BSP_USART_H
#define __BSP_USART_H

#include"usart.h"
#include"BSP_Chassis.h"


#define chassis_data_analyze() \
do{ \
		chassis_motor1.vpid.target_speed=gdata[0];    \
    chassis_motor2.vpid.target_speed=gdata[1];    \
		chassis_motor3.vpid.target_speed=gdata[2];	  \
		chassis_motor4.vpid.target_speed=gdata[3];	  \
	}while(0)                                   \



void RX_chassis_speed(void);

#endif

