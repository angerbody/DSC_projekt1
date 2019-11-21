/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l1xx.h"
			

int main(void)
{

	RCC -> AHBENR |= (RCC_AHBENR_GPIOAEN);
	GPIOA -> MODER = 0xA8000000; //reset value
	GPIOA -> MODER |= (GPIO_MODER_MODER5_0);
	GPIOA -> MODER &= ~(GPIO_MODER_MODER5_1);

	GPIOA -> ODR |= (GPIO_ODR_ODR_5);

	for(;;);
}
