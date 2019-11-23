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

uint8_t i = 0;

int main(void)
{

	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;

	GPIOA -> MODER = 0xA8000000; //reset value
	GPIOA -> MODER |= GPIO_MODER_MODER5_1;
	GPIOA -> MODER &= ~GPIO_MODER_MODER5_0;
	GPIOA -> AFR[0] = 0x00100000; //GPIO_AFRL_AFSEL5; --AF1 - TIM2_CH1 --NA SZTYWNO, MAKRO DO SPRAWDZENIA

	TIM2 -> PSC = 50;
	TIM2 -> ARR = 255;
	TIM2 -> CCR1 = 50;
	TIM2 -> CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1PE;
	TIM2 -> CCER |= TIM_CCER_CC1E;
	TIM2 -> EGR |= TIM_EGR_UG;
	TIM2 -> CR1 |= TIM_CR1_CEN;

	while (1)
	{
		TIM2 -> CCR1 = i++;
		for (int b = 0; b < 10; b++)
		{

		}//PRYMITYWNE OPӏNIENIE
	}
}
