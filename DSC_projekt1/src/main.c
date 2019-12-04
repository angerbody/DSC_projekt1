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
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_hal.h"

uint16_t i = 0;
uint8_t dir = 1;
uint8_t fadeFlag = 1;
uint8_t blinkFlag = 0;
uint16_t PomiarADC;

ADC_HandleTypeDef adc;
//TIM_Handle_TypeDef tim_hal = {};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc)
{
    PomiarADC = HAL_ADC_GetValue(adc);
}

void ADC_IRQHandler()
{
    HAL_ADC_IRQHandler(&adc);
}

int main(void)
{
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

	LL_USART_InitTypeDef usart = {0};
	LL_GPIO_InitTypeDef usart_pins = {0};

	usart_pins.Pin = LL_GPIO_PIN_2;
	usart_pins.Mode = LL_GPIO_MODE_ALTERNATE;
	usart_pins.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	usart_pins.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	usart_pins.Pull = LL_GPIO_PULL_UP;
	usart_pins.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA, &usart_pins);

	usart_pins.Pin = LL_GPIO_PIN_3;
	usart_pins.Mode = LL_GPIO_MODE_ALTERNATE;
	usart_pins.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	usart_pins.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	usart_pins.Pull = LL_GPIO_PULL_UP;
	usart_pins.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOA, &usart_pins);

	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(USART2_IRQn);

	usart.BaudRate = 9600;
	usart.DataWidth = LL_USART_DATAWIDTH_8B;
	usart.StopBits = LL_USART_STOPBITS_1;
	usart.Parity = LL_USART_PARITY_NONE;
	usart.TransferDirection = LL_USART_DIRECTION_TX_RX;
	usart.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	usart.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &usart);
	LL_USART_DisableIT_CTS(USART2);
	LL_USART_EnableIT_RXNE( USART2 );
	LL_USART_ConfigAsyncMode(USART2);
	LL_USART_Enable(USART2);

	LL_USART_ReceiveData8(USART2);


	///////////////////////////
	// Hardware Abstraction Layer
	///////////////////////////
	HAL_Init();
	__HAL_RCC_ADC1_CLK_ENABLE();

	ADC_ChannelConfTypeDef adcChannel = {0};

	adc.Instance = ADC1;
	adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	adc.Init.Resolution = ADC_RESOLUTION_12B;
	adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	adc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
	adc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
	adc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
	adc.Init.ContinuousConvMode = ENABLE;
	adc.Init.NbrOfConversion = 1;
	adc.Init.DiscontinuousConvMode = DISABLE;
	adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	adc.Init.DMAContinuousRequests = DISABLE;
	HAL_ADC_Init(&adc);

	adcChannel.Channel = ADC_CHANNEL_TEMPSENSOR;
	adcChannel.Rank = ADC_REGULAR_RANK_1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_4CYCLES;
	HAL_ADC_ConfigChannel(&adc, &adcChannel);

	//NVIC_EnableIRQ(ADC1_IRQn);
	HAL_ADC_Start_IT(&adc);

	while (1)
	{

		 //if (HAL_ADC_PollForConversion(&adc, 1000000) == HAL_OK) {

		 //PomiarADC = HAL_ADC_GetValue(&adc);
		 //Vsense = (3.3*PomiarADC)/4095.0;
		 //Temperature = ((Vsense-0.76)/0.0025)+25;
		 //}
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

void USART2_IRQHandler(void)
{
	if (USART2->DR == 'b')
	{
		blinkFlag = 1;
		fadeFlag = 0;
	}
	else if (USART2->DR == 'f')
	{
		blinkFlag = 0;
		fadeFlag = 1;
	}
	else if (USART2->DR == 't')
	{

	}

	LL_USART_ReceiveData8(USART2);

}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
	//if(htim->Instance == TIM2)
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}

