#include "MCP.h"

extern uint8_t frameBuff[MAX_PAYLOAD_LENGTH+2];
extern uint8_t flag;

uint8_t mcpCalculateCRC(mcpFrame* frame)
{
	uint16_t total = frame->frameStart + frame->payloadLength;
	uint8_t i=0;
	while(i!=frame->payloadLength)
	{
		total+=frame->payload[i];
		i++;
	}
	return (uint8_t)((total>>8) + (total&0xFF));
}

mcpErrorCode mcpSetRegister(MC_Protocol_REG_t reg,void* data,uint8_t dataSize,USART_TypeDef * UART,mcpSelectedMotor motor)
{
	mcpFrame* frame = (void*)frameBuff;
	uint8_t crc;
	frame->frameStart = motor<<5 | SET_REGISTER;
	frame->payloadLength=dataSize+1;
	*frame->payload=(uint8_t)reg;
	memcpy(frame->payload+1,data,dataSize);
	crc=mcpCalculateCRC(frame);
	frame->payload[dataSize+1]=crc;
	uartTransmittBuff((uint8_t*)frame,dataSize+4,UART);
	if(uartRecive(UART))
	{
		if(frame->frameStart==0xF0)
		{
			return ERROR_NONE;
		}
		else
		{
			return *frame->payload;
		}
	}
	else
	{
		return ERROR_NOT_RECIVE;
	}
}

mcpErrorCode mcpCommand(mcpCommandCode command,USART_TypeDef * UART,mcpSelectedMotor motor)
{
	mcpFrame* frame = (void*)frameBuff;
	uint8_t crc;
	frame->frameStart = motor<<5 | EXECUTE_COMMAND;
	frame->payloadLength=0x01;
	frame->payload[0]=(uint8_t)command;
	crc=mcpCalculateCRC(frame);
	frame->payload[1]=(uint8_t)crc;
	uartTransmittBuff((uint8_t*)frame,4,UART);
	if(uartRecive(UART))
	{
		if(frame->frameStart==0xF0)
		{
			return ERROR_NONE;
		}
		else
		{
			return *frame->payload;
		}
	}
	else
	{
		return ERROR_NOT_RECIVE;
	}
}

mcpFrame* mcpGetRegister(MC_Protocol_REG_t reg,USART_TypeDef * UART,mcpSelectedMotor motor)
{
	
	mcpFrame* frame = (void*)frameBuff;
	uint8_t crc,i;
	frame->frameStart = motor<<5 | GET_REGISTER;
	frame->payloadLength=1;
	*frame->payload=(uint8_t)reg;
	crc=mcpCalculateCRC(frame);
	frame->payload[1]=crc;
	DMA2_Stream2->CR&=~DMA_SxCR_EN;//Включить DMA
	DMA2_Stream2->NDTR =MAX_PAYLOAD_LENGTH;//Кол-во данных для передачи
	DMA2_Stream2->CR|=DMA_SxCR_EN;//Включить DMA
	uartTransmittBuff((uint8_t*)frame,4,UART);
	if(uartRecive(UART))
	{
		return frame;
	}
	frame->payload[0]=ERROR_NOT_RECIVE;
	return frame;
}

mcpErrorCode mcpRamp(uint32_t finalSpeed, uint16_t duration,USART_TypeDef * UART,mcpSelectedMotor motor)
{
	mcpFrame* frame = (void*)frameBuff;
	uint8_t crc;
	frame->frameStart = motor<<5 | EXEC_RAMP;
	frame->payloadLength=6;
	memcpy(frame->payload,&finalSpeed,4);
	memcpy(frame->payload+4,&duration,2);
	crc=mcpCalculateCRC(frame);
	frame->payload[6]=(uint8_t)crc;
	uartTransmittBuff((uint8_t*)frame,9,UART);
	if(uartRecive(UART))
	{
		if(frame->frameStart==0xF0)
		{
			return ERROR_NONE;
		}
		else
		{
			return *frame->payload;
		}
	}
}

mcpErrorCode mcpSetCurrentRef(uint16_t Iq,uint16_t Id,USART_TypeDef * UART,mcpSelectedMotor motor)
{
	mcpFrame* frame = (void*)frameBuff;
	uint8_t crc;
	frame->frameStart = motor<<5 | SET_CURRENT_REFERENCE;
	frame->payloadLength=4;
	memcpy(frame->payload,&Iq,2);
	memcpy(frame->payload+2,&Id,2);
	crc=mcpCalculateCRC(frame);
	frame->payload[4]=(uint8_t)crc;
	uartTransmittBuff((uint8_t*)frame,7,UART);
	if(uartRecive(UART))
	{
		if(frame->frameStart==0xF0)
		{
			return ERROR_NONE;
		}
		else
		{
			return *frame->payload;
		}
	}
}