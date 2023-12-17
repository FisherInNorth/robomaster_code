#include "bsp_led.h"



void LED_Blue(unsigned char x)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, (GPIO_PinState)x);
}

void LED_Green(unsigned char x)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, (GPIO_PinState)x);
}

void LED_Red(unsigned char x)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, (GPIO_PinState)x);
}

void LED_Blue_Toggle()
{
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
}

void LED_Green_Toggle()
{
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
}

void LED_Red_Toggle()
{
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
}
