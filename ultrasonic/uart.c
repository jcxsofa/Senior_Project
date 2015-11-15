#include "uart.h"

void uart_gpio_init(void) {
	
	// ENABLE GPIOA CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// CONFIGURE GPIOA PINS 9 AND 10 AS ALTERNATE FUNCTION
	GPIOA->MODER |= ((0x2 << 9*2) | (0x2 << 10*2));
	
	// CONFIGURE ALTERNATE FUNCTIONS AS USART1
	GPIOA->AFR[1] |= ((0x7 << 1*4) | (0x7 << 2*4));
	
	// CONFIGURE PIN 10 AS OPEN DRAIN
	GPIOA->OTYPER |= (1 << 10);
	
	// CONFIGURE OUTPUT SPEED TO 40MHz
	GPIOA->OSPEEDR |= ((0x3 << 9*2) | (0x3 << 10*2));
	
	// CONFIGURE PINS 9 AND 10 AS NO PULLUP/PULLDOWN
	GPIOA->PUPDR  &= 0xFFC3FFFF;
	
	
}

void USART_Init(USART_TypeDef * USARTx) {
	
	// CONFIGURE HC-05 DEFAULTS
	// 8 DATA BITS, 9600 BAUD, 1 STOP BIT, NO PARITY
	
	// ENABLE USART1 CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	// CONFIGURE 8 BIT WORD LENGTH
	USARTx->CR1 &= ~USART_CR1_M;
	
	// CONFIGURE OVERSAMPLING OF 16 TIMES BAUD RATE
	USARTx->CR1 &= ~USART_CR1_OVER8;
	
	// CONFIGURE A SINGLE STOP BIT
	USARTx->CR2 &= ~USART_CR2_STOP;
	
	// CONFIGURE 9600 BAUD RATE
	USARTx->BRR = 0x683;
	
	// ENABLE TRANSMITTER AND RECIEVER
	USARTx->CR1= (USART_CR1_RE | USART_CR1_TE);
	
	// ENABLE USART
	USARTx->CR1 |= USART_CR1_UE;
	
	// ENABLE RECIEVED DATA READY INTERRUPT
	USARTx->CR1 |= USART_CR1_RXNEIE;
	
	// SET PRIORITY AND ENABLE INTERRUPT
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	
}

void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes) {
	int i;
		
	// TXE IS CLEARED BY A WRITE TO THE USART_DR REGISTER
	// TXE IS SET BY HARDWARE WHEN THE CONTENT OF THE TDR
	// REGISTER HAS BEEN TRANSFERRED INTO THE SHIFT REGISTER
	
	for(i=0; i < nBytes; i++) {
		// WAIT UNTIL TXE (TX enpty) IS SET
		// WRITING USART_DR AUTOMATICALLY CLEARS THE TXE FLAG
		while(!(USARTx->SR & USART_SR_TXE));
		USARTx->DR = (buffer[i] & 0x1FF);	
	}
	
	// WAIT UNTIL TC BIT IS SET
	while(!(USARTx->SR & USART_SR_TC));
	USARTx->SR &= ~USART_SR_TC;
}

void USART_IRQHandler(USART_TypeDef * USARTx,
											uint8_t * buffer,
											uint8_t * pRx_counter) {
	
	if(USARTx->SR & USART_SR_RXNE) {
		// going to try resetting counter to always begin at beginning of buffer
		//(*pRx_counter) = 0;
		
		// READING USART_DR WILL ALSO CLEAR THE RXNE FLAG
		buffer[*pRx_counter] = USARTx->DR;
		if(buffer[*pRx_counter] == 'i') *pRx_counter = 0;
		else	(*pRx_counter)++;
		
		if((*pRx_counter) >= BufferSize)
			(*pRx_counter) = 0;
	}
}



