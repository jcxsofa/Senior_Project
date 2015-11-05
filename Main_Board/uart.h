#ifndef UART_H_
#define UART_H_

#include "stm32f407xx.h"
#include <stdlib.h>

#define BufferSize 32

uint8_t USART1_Buffer_Rx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART1_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t Rx1_Counter = 7;
int newspeed;

void uart_gpio_init(void);
void USART_Init(USART_TypeDef * USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void USART_IRQHandler(USART_TypeDef * USARTx,
											uint8_t * buffer,
											uint8_t * pRx_counter);
void USART1_IRQHandler(void);
void decode(uint8_t * buffer);

#endif // UART_H_