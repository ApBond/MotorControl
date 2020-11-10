#include "MotorControl.h"

mcpErrorCode startMotor(USART_TypeDef * UART)
{
	return mcpCommand(START_MOTOR,UART,MOTOR_1);
}

mcpErrorCode stopMotor(USART_TypeDef * UART)
{
	return mcpCommand(STOP_MOTOR,UART,MOTOR_1);
}

mcpErrorCode faultAck(USART_TypeDef * UART)
{
	return mcpCommand(FAULT_ACK,UART,MOTOR_1);
}

mcpErrorCode setSpeed(uint32_t finalSpeed, uint16_t duration,USART_TypeDef * UART)
{
	return mcpRamp(finalSpeed,duration,UART,MOTOR_1);
}

mcpErrorCode setCurrent(uint16_t Iq,uint16_t Id,USART_TypeDef * UART)
{
	return mcpSetCurrentRef(Iq,Id,UART,MOTOR_1);
}

mcpErrorCode setSpeedPID(uint16_t kp,uint16_t ki,uint16_t kd,USART_TypeDef * UART)
{
	mcpErrorCode errorCode = mcpSetRegister(MC_PROTOCOL_REG_SPEED_KP,&kp,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) return errorCode;
	errorCode=mcpSetRegister(MC_PROTOCOL_REG_SPEED_KI,&ki,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) return errorCode;
	errorCode=mcpSetRegister(MC_PROTOCOL_REG_SPEED_KD,&kd,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) 
		return errorCode;
	else
		return ERROR_NONE;
}

mcpErrorCode setTorquePID(uint16_t kp,uint16_t ki,uint16_t kd,USART_TypeDef * UART)
{
	mcpErrorCode errorCode = mcpSetRegister(MC_PROTOCOL_REG_TORQUE_KP,&kp,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) return errorCode;
	errorCode=mcpSetRegister(MC_PROTOCOL_REG_TORQUE_KI,&ki,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) return errorCode;
	errorCode=mcpSetRegister(MC_PROTOCOL_REG_TORQUE_KD,&kd,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) 
		return errorCode;
	else
		return ERROR_NONE;
}

mcpErrorCode setFluxPID(uint16_t kp,uint16_t ki,uint16_t kd,USART_TypeDef * UART)
{
	mcpErrorCode errorCode = mcpSetRegister(MC_PROTOCOL_REG_FLUX_KP,&kp,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) return errorCode;
	errorCode=mcpSetRegister(MC_PROTOCOL_REG_FLUX_KI,&ki,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) return errorCode;
	errorCode=mcpSetRegister(MC_PROTOCOL_REG_FLUX_KD,&kd,2,UART,MOTOR_1);
	if(errorCode!=ERROR_NONE) 
		return errorCode;
	else
		return ERROR_NONE;
}

mcpErrorCode getSpeed(USART_TypeDef * UART,int32_t* speedBuf)
{
	mcpFrame* frame;
	int32_t speed;
	frame = mcpGetRegister(MC_PROTOCOL_REG_SPEED_MEAS,UART,MOTOR_1);
	if(frame->frameStart==0xF0)
	{
		memcpy(speedBuf,frame->payload,4);
		return ERROR_NONE;
	}
	return frame->payload[0];
}

mcpErrorCode getMotorPower(USART_TypeDef * UART,uint16_t* powerBuff)
{
	mcpFrame* frame;
	frame = mcpGetRegister(MC_PROTOCOL_REG_MOTOR_POWER,UART,MOTOR_1);
	if(frame->frameStart==0xF0)
	{
		memcpy(powerBuff,frame->payload,2);
		return ERROR_NONE;
	}
	return frame->payload[0];
}

mcpErrorCode getMotorState(USART_TypeDef * UART,uint8_t* stateBuff)
{
	mcpFrame* frame;
	frame = mcpGetRegister(MC_PROTOCOL_REG_STATUS,UART,MOTOR_1);
	if(frame->frameStart==0xF0)
	{
		*stateBuff=*frame->payload;
		return ERROR_NONE;
	}
	return frame->payload[0];
}

mcpErrorCode getTorque(USART_TypeDef * UART,int16_t* torqueBuf)
{
	mcpFrame* frame;
	int32_t speed;
	frame = mcpGetRegister(MC_PROTOCOL_REG_TORQUE_MEAS,UART,MOTOR_1);
	if(frame->frameStart==0xF0)
	{
		memcpy(torqueBuf,frame->payload,2);
		return ERROR_NONE;
	}
	return frame->payload[0];
}

mcpErrorCode getFlux(USART_TypeDef * UART,int16_t* fluxBuf)
{
	mcpFrame* frame;
	int32_t speed;
	frame = mcpGetRegister(MC_PROTOCOL_REG_FLUX_MEAS,UART,MOTOR_1);
	if(frame->frameStart==0xF0)
	{
		memcpy(fluxBuf,frame->payload,2);
		return ERROR_NONE;
	}
	return frame->payload[0];
}