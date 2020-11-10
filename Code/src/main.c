#include "main.h"

#define MEASURE_PERIOD 100
#define MOTOR USART6

uint8_t motorState=0;
uint8_t changeSpeed=0;
uint8_t measureTimeFlag=0;
uint8_t changeSpeedTimeFlag=0;
int32_t refSpeed;
int32_t speed;
uint8_t state;
int16_t torque,flux;
uint16_t motorPower=0;
int32_t lastSpeed=0;
mcpErrorCode error;

void EXTI15_10_IRQHandler(void)
{
	EXTI->PR|=EXTI_PR_PR13;
	changeSpeed=1;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM10->SR & TIM_SR_UIF)
	{
		TIM10->SR&=~TIM_SR_UIF;//Сброс флага прерывания
		measureTimeFlag=1;
	}
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if(TIM11->SR & TIM_SR_UIF)
	{
		TIM11->SR&=~TIM_SR_UIF;//Сброс флага прерывания
		changeSpeedTimeFlag=1;
	}
}

__INLINE int32_t abs(int32_t num)
{
	if(num>=0)
		return num;
	else
		return num*(-1);
}

__INLINE void buttonInit(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;//Тактирование порта C
	GPIOC->PUPDR|=GPIO_PUPDR_PUPD13_0;//PC13 Pull up
	NVIC_EnableIRQ(EXTI15_10_IRQn);//Включить прерывание
	SYSCFG->EXTICR[3]|=SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->IMR|=EXTI_IMR_IM13;
	EXTI->FTSR|=EXTI_FTSR_TR13;
}

__INLINE void Tim10Init(void)
{
	RCC->APB2ENR|=RCC_APB2ENR_TIM10EN;//Включить тактирование таймера-счетчика 10
	TIM10->PSC=10000;//Делитель на 10КГц
	TIM10->CR1|=TIM_CR1_CEN;//Включить таймер
	TIM10->ARR=MEASURE_PERIOD*10;//Считаем до конца
	TIM10->DIER|=TIM_DIER_UIE;//Включить прерывание по пререполнению таймера 10
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);//Разрешить прерывание от таймера 10
	TIM10->CR1|=TIM_CR1_CEN;//Включить таймер 10
}

int main()
{
	
	RccClockInit();
	delayInit(); 
	buttonInit();
	encoderInit();
	Tim10Init();
	uart6Init(100000000,9600);
	uart2Init(100000000,115200);
	uart1Init(100000000,9600);
	setSpeedPID(1000,1000,500,MOTOR);
	setSpeed(4000,100,MOTOR);
	lastSpeed=4000;
	TIM2->CNT=900 ;
	while(1)
	{
		if(changeSpeed)
		{
			if(!motorState)
			{
				if(startMotor(MOTOR)==ERROR_NONE) motorState=1;
			}
			else
			{
				if(stopMotor(MOTOR)==ERROR_NONE) motorState=0;
			}
			changeSpeed=0;
		}
		if(TIM2->CNT>1000) TIM2->CNT=1000;
		if(TIM2->CNT<10) TIM2->CNT=10;
		refSpeed=(TIM2->CNT*10)-5000;
		if(lastSpeed!=refSpeed)
		{

			if(abs(refSpeed)<1500 && abs(lastSpeed)>=1500)
			{
				setSpeedPID(1000,1000,500,MOTOR);
			}
			if(abs(refSpeed)>1500 && abs(refSpeed)<3000)
			{
				if(abs(lastSpeed)<=1500 || abs(lastSpeed)>=3000)
					setSpeedPID(2100,1750,700,MOTOR);
			}
			if(abs(refSpeed)>3000 && abs(lastSpeed)<=3000)
			{
				setSpeedPID(4000,2850,1500,MOTOR);
			}
			
			if(setSpeed(refSpeed,100,MOTOR)==ERROR_NONE)
			{
				lastSpeed=refSpeed;
			}
		}
		if(measureTimeFlag)
		{
			getSpeed(MOTOR,&speed);
			getMotorState(MOTOR,&state);
			getTorque(MOTOR,&torque);
			getFlux(MOTOR,&flux);
			uartTransmittBuff((uint8_t*)&speed,4,USART2);
			uartTransmittBuff((uint8_t*)&refSpeed,4,USART2);
			measureTimeFlag=0;
		}
	}
}
