#include "uart.h"

void uart_gpio_init(void) {
	
		// ENABLE GPIOD CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	// ENABLE USART1 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	
	// CONFIGURE GPIOD PINS 5 AND 6 AS ALTERNATE FUNCTION
	GPIOD->MODER |= ((0x2 << 5*2) | (0x2 << 6*2));
	
	// CONFIGURE ALTERNATE FUNCTIONS AS USART2
	GPIOD->AFR[0] |= ((0x7 << 5*4) | (0x7 << 6*4));
	
	// CONFIGURE PIN 6 AS OPEN DRAIN
	GPIOD->OTYPER |= (1 << 6);
	
	// CONFIGURE OUTPUT SPEED TO 40MHz
	GPIOD->OSPEEDR |= ((0x3 << 5*2) | (0x3 << 6*2));
	
	// CONFIGURE PINS 5 AND 6 AS NO PULLUP/PULLDOWN
	GPIOA->PUPDR  &= ~((1 << 5) | (1 << 6));
	
}

void USART_Init(USART_TypeDef * USARTx) {
	
	// CONFIGURE HC-05 DEFAULTS
	// 8 DATA BITS, 9600 BAUD, 1 STOP BIT, NO PARITY
	
	// CONFIGURE 8 BIT WORD LENGTH
	USARTx->CR1 &= ~USART_CR1_M;
	
	// CONFIGURE OVERSAMPLING OF 16 TIMES BAUD RATE
	USARTx->CR1 &= ~USART_CR1_OVER8;
	
	// CONFIGURE A SINGLE STOP BIT
	USARTx->CR2 &= ~USART_CR2_STOP;
	
	// CONFIGURE 115200 BAUD RATE
	USARTx->BRR = 139;
	
//	//CONFIGURE 115200 BAUD RATE
//	USARTx->BRR = 139;
//	
	// ENABLE TRANSMITTER AND RECIEVER
	USARTx->CR1= (USART_CR1_RE | USART_CR1_TE);
	
	// ENABLE USART
	USARTx->CR1 |= USART_CR1_UE;
	
	// ENABLE RECIEVED DATA READY INTERRUPT
	USARTx->CR1 |= USART_CR1_RXNEIE;
	
	// SET PRIORITY AND ENABLE INTERRUPT
	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);
	
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
											
void display_stats(struct Motor *M){
	
	char print[90];	
	
	sprintf(print, "M%1d BEMF = % 3.3frpm   ENC = % 3.3frpm   TARG = % 3.3frpm   ERR = % 3.3f\n\r", M->wheel, M->BEMF_Speed, M->Encoder_Speed, M->Current, M->Error);
	
	USART_Write(USART2, print, 78);
}



