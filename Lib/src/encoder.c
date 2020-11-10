#include "encoder.h"

uint16_t encoderCount;

void encoderInit(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Тактирование порта A
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;//Такстирование таймера 2
	GPIOA->MODER|=GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;//PA0,PA1 в режим алдьтернативных функцияй
	GPIOA->AFR[0]|=GPIO_AFRL_AFRL0_0 | GPIO_AFRL_AFRL1_0;//PA0,PA1 - AF1
	GPIOA->PUPDR|=GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0;//PA0,PA1 Pull up
	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;//Инвертированный режим (по срезу)
	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;//Сконфигурировать CH0,CH1 для работы по триггеру
	TIM2->SMCR = TIM_SMCR_SMS_1;//Режим энкодера по TI1
	TIM2->CCMR1|=0xE<<TIM_CCMR1_IC1F_Pos | 0xE<<TIM_CCMR1_IC2F_Pos;//Настройка входных фильтров
	TIM2->ARR = 1100;//Верхний предел счета
	TIM2->CR1|= TIM_CR1_CEN;//Старт таймера
}
