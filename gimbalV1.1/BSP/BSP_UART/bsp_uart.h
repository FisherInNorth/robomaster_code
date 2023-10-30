#ifndef __BSP_UART_H
#define __BSP_UART_H


#define DMA_REC_LEN    10

extern short int F_move_flag;
extern short int L_move_flag;
void uart_init(void);
void DMA_Send(void);

#endif

