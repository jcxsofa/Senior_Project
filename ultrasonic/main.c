#include "stm32l1xx.h"
#include "uart.h"
#define BufferSize 32
void clock_Configure(void);
void USART_Init(USART_TypeDef * USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void decode(uint8_t * buffer);

uint8_t USART1_Buffer_Rx[BufferSize];// = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART1_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, '\n',};
uint8_t Rx1_Counter = 0;


int main(void){
	
	// ENABLE HIGH SPEED CLOCK
	clock_Configure();
	
	// CONFIGURE UART
	uart_gpio_init();
	USART_Init(USART1);
	
	
	
	
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
	
	
}


void sensor_gpio_init(void) {
	
	//ENABLE GPIOB CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// 
	
	
	
	
}



