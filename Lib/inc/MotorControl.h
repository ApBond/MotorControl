#ifndef __MOTOR_CONTROL
#define __MOTOR_CONTROL

#include "stm32f411xe.h"
#include "bsp.h"
#include "MCP.h"
#include "delay.h"

mcpErrorCode startMotor(USART_TypeDef * UART);//Пуск двигателя
mcpErrorCode stopMotor(USART_TypeDef * UART);//Остановка двигателя
mcpErrorCode faultAck(USART_TypeDef * UART);//Сброс ошибки
mcpErrorCode setSpeedPID(uint16_t kp,uint16_t ki,uint16_t kd,USART_TypeDef * UART);//Настройка регулятора контура скорости
mcpErrorCode setTorquePID(uint16_t kp,uint16_t ki,uint16_t kd,USART_TypeDef * UART);//Настройка регулятора момента
mcpErrorCode setFluxPID(uint16_t kp,uint16_t ki,uint16_t kd,USART_TypeDef * UART);//Настройка регулятора потока
mcpErrorCode setSpeed(uint32_t finalSpeed, uint16_t duration,USART_TypeDef * UART);//Задание скорости
mcpErrorCode setCurrent(uint16_t Iq,uint16_t Id,USART_TypeDef * UART);//Залание тока
mcpErrorCode getSpeed(USART_TypeDef * UART,int32_t* speedBuf);//Чтение информации о скорости
mcpErrorCode getMotorPower(USART_TypeDef * UART,uint16_t* powerBuff);//Чтение информации о мощности 
mcpErrorCode getMotorState(USART_TypeDef * UART,uint8_t* stateBuff);//Чтение информации о состоянии
mcpErrorCode getTorque(USART_TypeDef * UART,int16_t* torqueBuf);//Чтение информации о моменте
mcpErrorCode getFlux(USART_TypeDef * UART,int16_t* fluxBuf);//Чтение информации о потоке



#endif

