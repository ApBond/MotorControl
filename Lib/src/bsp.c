#include "bsp.h"

uint8_t frameBuff[MAX_PAYLOAD_LENGTH+2];
uint8_t reciveStatus=0;

/////////////////////////////Прерывания UART для приёма данных//////////////////////////////////////////////

void USART1_IRQHandler(void)
{
	frameBuff[reciveStatus]=USART1->DR;
	reciveStatus++;
}

void USART2_IRQHandler(void)
{
	frameBuff[reciveStatus]=USART2->DR;
	reciveStatus++;
}

void USART6_IRQHandler(void)
{
	frameBuff[reciveStatus]=USART6->DR;
	reciveStatus++;
}

__STATIC_INLINE void Tim9Init(uint32_t coreFreq)//Таймер 9 используется для ожидания при чтении по UART
{
	RCC->APB2ENR|=RCC_APB2ENR_TIM9EN;//Включить тактирование таймера-счетчика 9
	TIM9->PSC=coreFreq/10000;//Делитель на 10КГц
	TIM9->CR1|=TIM_CR1_CEN;//Включить таймер
	TIM9->ARR=0xFFFF;//Считаем до конца
}

/////////////////////////////Настройка UART//////////////////////////////////////////////
void uart6Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PC6-TX
	//PC7-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;//Включить тактирвание порта C
	RCC->APB2ENR|=RCC_APB2ENR_USART6EN;//Включить тактирование UART6
	GPIOC->MODER|=GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;//PC6,PC7 в режим альтернативной функции
	GPIOC->AFR[0]|=GPIO_AFRL_AFRL6_3 | GPIO_AFRL_AFRL7_3;//Включить AF8
	GPIOC->PUPDR|=GPIO_PUPDR_PUPD7_0;//PC7 Pull up
	USART6->CR1 = 0;//Сбрасываем конфигурацию
	USART6->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART6->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART6_IRQn);
	USART6->CR1|=USART_CR1_UE;//Включить uart
}

void uart2Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PA2-TX
	//PA3-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Включить тактирвание порта А
	RCC->APB1ENR|=RCC_APB1ENR_USART2EN;//Включить тактирование UART2
	GPIOA->MODER|=GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1;//PA2,PA3 в режим альтернативной функции
	GPIOA->AFR[0]|=GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_2 | GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_2;//Включить AF7
	GPIOA->PUPDR|=GPIO_PUPDR_PUPD3_0;//PA3 Pull up
	USART2->CR1 = 0;//Сбрасываем конфигурацию
	USART2->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART2->BRR|=coreFreq/(2*baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR1|=USART_CR1_UE;//Включить uart
}

void uart1Init(uint32_t coreFreq, uint32_t baudRate)
{
	//PA9-TX
	//PA10-RX
	Tim9Init(coreFreq);
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;//Включить тактирвание порта А
	RCC->APB2ENR|=RCC_APB2ENR_USART1EN;//Включить тактирование  
	GPIOA->MODER|=GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;//PA9,PA10 в режим альтернативной функции
	GPIOA->AFR[1]|=GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_2 | GPIO_AFRH_AFRH2_0 | GPIO_AFRH_AFRH2_1 | GPIO_AFRH_AFRH2_2;//Включить AF7
	GPIOA->PUPDR|=GPIO_PUPDR_PUPD10_0;//PA3 Pull up
	USART1->CR1 = 0;//Сбрасываем конфигурацию
	USART1->CR1|=USART_CR1_RE|USART_CR1_TE;//Включить приём и передачу
	USART1->BRR|=coreFreq/(baudRate);//Скорость работы uart
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->CR1|=USART_CR1_UE;//Включить uart
}
/////////////////////////////Функции приёма-передачи//////////////////////////////////////////////

void uartTransmitt(uint8_t data,USART_TypeDef * UART)
{
	while (!(UART->SR & USART_SR_TXE)){}//Ждем, пока UART освлбодиться
	UART->DR = data;//Отправляем данные
}

void uartTransmittBuff(uint8_t* data,uint32_t size,USART_TypeDef * UART)
{
	while(size)
	{
		uartTransmitt(*data++,UART);
		size--;
	}
}

uint8_t uartRecive(USART_TypeDef * UART)
{
	reciveStatus=0;
	UART->CR1|=USART_CR1_RXNEIE;//Разрешить прерывание по приёму
	TIM9->CNT=0;
	while(reciveStatus!=frameBuff[1]+3)
	{
		if(TIM9->CNT>=UART_MAX_TIMEOUT_MS*10)
		{
			USART1->CR1&=~USART_CR1_RXNEIE;//Запретить прерывание по приёму
			return 0;//Если закончился тайм аут завершаем ожидание
		}
	}
	UART->CR1&=~USART_CR1_RXNEIE;//Запретить прерывание по приёму
	return 1;
}