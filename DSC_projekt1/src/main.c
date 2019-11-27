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

uint16_t i = 0;
uint8_t dir = 1;

int main(void)
{
	//Konfiguracja RCC
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Konfiguracja GPIO-PA5 - funkcja alternatywna (TIM2_CH1)
	GPIOA -> MODER = 0xA8000000; //reset value
	GPIOA -> MODER |= GPIO_MODER_MODER5_1;
	GPIOA -> MODER &= ~GPIO_MODER_MODER5_0;
	GPIOA -> AFR[0] = 0x00100000; //GPIO_AFRL_AFSEL5; --AF1 - TIM2_CH1 --NA SZTYWNO, MAKRO DO SPRAWDZENIA

	//Konfiguracja TIM3 - podstawa czasu (100ms ??)
	//SystemCoreClock        = 2097000U;
	TIM3 -> PSC = SystemCoreClock/1000; //podstawa czasu = 1ms
	TIM3 -> ARR = 100;
	TIM3 -> EGR |= TIM_EGR_UG;
	TIM3 -> DIER |= TIM_DIER_CC1IE;
	NVIC_EnableIRQ(TIM3_IRQn);
	TIM3 -> CR1 |= TIM_CR1_CEN;

	//Konfiguracja TIM2 - zegar z wyjœciem PWM
	TIM2 -> PSC = 100;
	TIM2 -> ARR = 10000;
	TIM2 -> CCR1 = 100;
	TIM2 -> CR1 |= TIM_CR1_ARPE;
	TIM2 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
	TIM2 -> CCER |= TIM_CCER_CC1E; //enable channel 1
	TIM2 -> EGR |= TIM_EGR_UG; //co to by³o, event generator???
	TIM2 -> CR1 |= TIM_CR1_CEN; //enable timer

	while (1)
	{

	}
}

void TIM3_IRQHandler(void)
{
	TIM3->SR &= ~TIM_SR_UIF;
	if (dir == 1)
	{
		TIM2 -> CCR1 = i++;
		if(i > (TIM2 -> ARR))
			dir = 0;
	}
	else
	{
		TIM2 -> CCR1 = i--;
		if (i == 0)
			dir = 1;
	}
}

