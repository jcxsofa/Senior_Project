#ifndef UART_H_
#define UART_H_

#include "stm32f407xx.h"
#include <stdlib.h>
#include <string.h>
#include "motor.h"

#define BufferSize 32

struct Remote_Data {
	
	// motor 1 data to transmit
	float M1_Current;
	float M1_BEMF_Speed;
	float M1_Encoder_Speed;
	float M1_Error;
	float M1_Desired_Speed;
	
	// motor 2 data to transmit
	float M2_Current;
	float M2_BEMF_Speed;
	float M2_Encoder_Speed;
	float M2_Error;
	float M2_Desired_Speed;
	
	// motor 3 data to transmit
	float M3_Current;
	float M3_BEMF_Speed;
	float M3_Encoder_Speed;
	float M3_Error;
	float M3_Desired_Speed;
	
	// motor 4 data to transmit
	float M4_Current;
	float M4_BEMF_Speed;
	float M4_Encoder_Speed;
	float M4_Error;
	float M4_Desired_Speed;
	
	// power stuff
	float Battery_Voltage;
	float Total_Current;
	
};


void uart_gpio_init(void);
void USART_Init(USART_TypeDef * USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void USART_IRQHandler(USART_TypeDef * USARTx,
											uint8_t * buffer,
											uint8_t * pRx_counter);
void display_stats(struct Motor *M);

#endif // UART_H_