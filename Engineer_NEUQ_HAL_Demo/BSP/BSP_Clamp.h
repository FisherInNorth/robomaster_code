#ifndef __BSP_CLAMP_H
#define __BSP_CLAMP_H

#include "main.h"

#define Clamp_ON 				HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET)
#define Clamp_OFF 			HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET)
#define Clamp_Judge     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)

void AirMineral_Task(void);

#endif
