#include "stm32l1xx.h"
#define BufferSize 32
void clock_Configure(void);
void USART_Init(USART_TypeDef * USARTx);
void gpio_Configure(void);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void decode(uint8_t * buffer);

uint8_t USART1_Buffer_Rx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART1_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t Rx1_Counter = 7;

int main(void) {
	int i;
	// INITIALIZE USART CLOCK AND HIGH SPEED SYSTEM CLOCK
	clock_Configure();
	
	// INITIALIZE USART GPIO PINS
	gpio_Configure();
	
	// INITIALIZE USART1
	USART_Init(USART1);
	
	// SET PRIORITY AND ENABLE INTERRUPT
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	
	// TOGGLE PB6 ON
	GPIOB->ODR ^= (1<<6);
	
	// CONTINUOUSLY SEND "WORKING"
	while(1) {
		
//		for(i=0; i<10000000;) {
//			i = i + 1;
//		}
//		USART_Write(USART1, USART1_Buffer_Rx, Rx1_Counter);
	}
	
}

void clock_Configure(void) {
	
	// ENABLE HSI
	RCC->CR |= RCC_CR_HSION;
	
	// WAIT FOR HSI READY
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	
	// SELECT HSI AS SYSTEM CLOCK
	RCC->CFGR &= ~RCC_CFGR_SW_HSI;
	RCC->CFGR |= RCC_CFGR_SW_HSI;
	
	// WAIT FOR HSI TO BE SELECTED
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
	
	// ENABLE GPIOA CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// ENABLE USART1 CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	// FOR TESTING COMMUNICATION
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	
}

void gpio_Configure(void) {
	
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
	
	
	
	
	// CONFIGURE GPIOB PINS 9 AND 10 AS ALTERNATE FUNCTION
	GPIOB->MODER |= ((0x1 << 6*2) | (0x1 << 7*2));
	
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
	
	// CONFIGURE 9600 BAUD RATE
	USARTx->BRR = 0x683;
	
	// ENABLE TRANSMITTER AND RECIEVER
	USARTx->CR1= (USART_CR1_RE | USART_CR1_TE);
	
	// ENABLE USART
	USARTx->CR1 |= USART_CR1_UE;
	
	// ENABLE RECIEVED DATA READY INTERRUPT
	USARTx->CR1 |= USART_CR1_RXNEIE;
	
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
		(*pRx_counter) = 0;
		
		// READING USART_DR WILL ALSO CLEAR THE RXNE FLAG
		buffer[*pRx_counter] = USARTx->DR;
		(*pRx_counter)++;
		if((*pRx_counter) >= BufferSize)
			(*pRx_counter) = 0;
	}
}


void decode(uint8_t * buffer) {
	
	if( buffer[0] == 0x36) GPIOB->ODR ^= (1<<6);
	if( buffer[0] == 0x37) GPIOB->ODR ^= (1<<7);
	
}

void USART1_IRQHandler(void) {
	USART_IRQHandler(USART1, USART1_Buffer_Rx, &Rx1_Counter);
	decode(USART1_Buffer_Rx);
}



