#include "stm32f407xx.h"
#define BufferSize 32
void clock_Configure(void);
void USART_Init(USART_TypeDef * USARTx);
void gpio_Configure(void);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void decode(uint8_t * buffer);

uint8_t USART2_Buffer_Rx[BufferSize];// = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART2_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, '\n',};
uint8_t Rx2_Counter = 0;

int main(void) {
	int i;
	// INITIALIZE USART CLOCK AND HIGH SPEED SYSTEM CLOCK
	clock_Configure();
	
	// INITIALIZE USART GPIO PINS
	gpio_Configure();
	
	// INITIALIZE USART1
	USART_Init(USART2);
	
	// SET PRIORITY AND ENABLE INTERRUPT
	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);
	
	// TOGGLE PB6 ON
	GPIOB->ODR ^= (1<<6);
	
	// CONTINUOUSLY SEND "WORKING"
	while(1) {
		
//		for(i=0; i<1000000;) {
//			i = i + 1;
//		}
		//USART_Write(USART1, USART1_Buffer_Tx, 8);
	}
	
}

void clock_Configure(void) {
	
//	// ENABLE HSI
//	RCC->CR |= RCC_CR_HSION;
//	
//	// WAIT FOR HSI READY
//	while((RCC->CR & RCC_CR_HSIRDY) == 0);
//	
//	// SELECT HSI AS SYSTEM CLOCK
//	RCC->CFGR &= ~RCC_CFGR_SW_HSI;
//	RCC->CFGR |= RCC_CFGR_SW_HSI;
//	
//	// WAIT FOR HSI TO BE SELECTED
//	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
	
	
}

void gpio_Configure(void) {
	
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
		//(*pRx_counter) = 0;
		
		// READING USART_DR WILL ALSO CLEAR THE RXNE FLAG
		buffer[*pRx_counter] = USARTx->DR;
		if(buffer[*pRx_counter] == 'i') *pRx_counter = 0;
		else	(*pRx_counter)++;
		
		if((*pRx_counter) >= BufferSize)
			(*pRx_counter) = 0;
	}
}


void decode(uint8_t * buffer) {
	int i = 0;
	if( Rx2_Counter == 7 ) {
		for (i=0; i<=7; i++) {
			USART2_Buffer_Tx[i] = buffer[i];
		}
		USART_Write(USART2, USART2_Buffer_Tx, 7);
	}
	
}

void USART2_IRQHandler(void) {
	USART_IRQHandler(USART2, USART2_Buffer_Rx, &Rx2_Counter);
	decode(USART2_Buffer_Rx);
}



