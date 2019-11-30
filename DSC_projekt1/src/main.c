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
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_hal.h"

uint16_t i = 0;
uint8_t dir = 1;
uint8_t fadeFlag = 1;
uint8_t blinkFlag = 0;
//TIM_Handle_TypeDef tim_hal = {};

int main(void)
{
	HAL_Init();

	//__HAL_RCC_GPIOA_CLK_ENABLE();
	//GPIO_InitTypeDef gpio_hal = {0};
	//gpio_hal.Pin = GPIO_PIN_5;
	//gpio_hal.Mode = GPIO_MODE_OUTPUT_PP;
	//HAL_GPIO_Init(GPIOA, &gpio_hal);

	//__HAL_RCC_TIM2_CLK_ENABLE();
	//tim_hal.Instance = TIM2;
	//tim_hal.Init.Prescaler = SystemCoreClock/1000;
	//tim_hal.Init.Period = 500;
	//HAL_TIM_Base_Init(&tim_hal);
	//HAL_TIM_Base_Start_IT(&tim_hal);

	//HAL_NVIC_EnableIRQ(TIM2_IRQn)
	///////////////////////////
	// CMSIS
	///////////////////////////
	//Konfiguracja RCC
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Konfiguracja GPIO-PA5 - funkcja alternatywna (TIM2_CH1)
	GPIOA -> MODER = 0xA8000000; //reset value
	GPIOA -> MODER |= GPIO_MODER_MODER5_1;
	GPIOA -> MODER &= ~GPIO_MODER_MODER5_0;
	GPIOA -> AFR[0] = 0x00100000; //GPIO_AFRL_AFSEL5; --AF1 - TIM2_CH1 --NA SZTYWNO, MAKRO DO SPRAWDZENIA

	//Konfiguracja TIM3 - podstawa czasu
	//SystemCoreClock        = 2097000U;
	TIM3 -> PSC = SystemCoreClock/1000; //podstawa czasu = 1ms
	TIM3 -> ARR = 1;
	TIM3 -> EGR |= TIM_EGR_UG;
	TIM3 -> DIER |= TIM_DIER_CC1IE;
	TIM3 -> CR1 |= TIM_CR1_CEN;

	//Konfiguracja TIM2 - zegar z wyjœciem PWM
	TIM2 -> PSC = 10;
	TIM2 -> ARR = 300;
	TIM2 -> CCR1 = 0;
	TIM2 -> CR1 |= TIM_CR1_ARPE;
	TIM2 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
	TIM2 -> CCER |= TIM_CCER_CC1E; //enable channel 1
	TIM2 -> EGR |= TIM_EGR_UG; //co to by³o, event generator???
	TIM2 -> CR1 |= TIM_CR1_CEN; //enable timer

	//Koniguracja NVIC
	NVIC_EnableIRQ(TIM3_IRQn);


	///////////////////////////
	// Low Layer
	///////////////////////////
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_USART_InitTypeDef usart2;
	LL_USART_StructInit(&usart2);
	LL_USART_Init(USART2, &usart2);
	usart2.BaudRate = 9600;
	LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
	LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_0_5);

	//LL_USART_EnableIT_;

	while (1)
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(500UL);
	}
}

void TIM3_IRQHandler(void)
{
	if (fadeFlag == 1)
	{
		if (TIM2 -> PSC != 10)
			TIM2 -> PSC = 10;
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
	else if (blinkFlag == 1)
	{
		if ((TIM2 -> PSC != 10000) & (TIM2 -> CCR1 != (TIM2 -> ARR)/2))
		{
			TIM2 -> PSC = 10000;
			TIM2 -> CCR1 = (TIM2 -> ARR)/2;
		}
	}

	TIM3->SR = 0;
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
	//if(htim->Instance == TIM2)
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}

