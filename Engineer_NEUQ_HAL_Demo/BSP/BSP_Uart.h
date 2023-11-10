#ifndef __UART_H
#define __UART_H
#include "main.h"
#include "BSP_Motor.h"

void Usart_SendString(uint8_t *str);
void RC_CH_Send(uint16_t x,uint16_t y,uint16_t r,uint16_t i);
void RC_Chassis_Speed_Send(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4);
void UART_IdleIT_init(void);
void RC_Rescue_Move_Send(MOTOR_MOVE_t rescue_move);

#endif

