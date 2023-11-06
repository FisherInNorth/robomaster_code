#ifndef __UART_H
#define __UART_H
#include "main.h"

void Usart_SendString(uint8_t *str);
void RC_CH_Send(uint16_t x,uint16_t y,uint16_t r,uint16_t i);
void RC_Chassis_Speed_Send(uint16_t speed1,uint16_t speed2,uint16_t speed3,uint16_t speed4);
void UART_IdleIT_init(void);

#endif

