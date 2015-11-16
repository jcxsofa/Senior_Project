#include "stm32l1xx.h"
#include "uart.h"
#include <string.h>
#define BufferSize 32

void clock_Configure(void);
void tim_3_configure(void);
void tim_4_configure(void);
void sensor_gpio_init(void);

void USART_Init(USART_TypeDef * USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void decode(uint8_t * buffer);

uint8_t USART1_Buffer_Rx[BufferSize];// = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART1_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, '\n',};
uint8_t Rx1_Counter = 0;


int main(void){
	
	int value = 0;
	int old_value = 0;
	int new_value = 0;
	float distance = 0;
	float speed = 0;
	char print[50];
	
	// ENABLE HIGH SPEED CLOCK
	clock_Configure();
	
	// CONFIGURE SENSOR GPIO
	sensor_gpio_init();
	
	// CONFIGURE TIMERS
	tim_4_configure();
	tim_3_configure();
	
	// CONFIGURE UART
	uart_gpio_init();
	USART_Init(USART1);
	
	while(1) {
		
		// RESET TIMER 4 INTERRUPT
		TIM4->SR &= ~(1 << 1);
		
		// RESET COUNTER
		TIM4->CNT &= 0;
		
		// ENABLE TIMER 3 TO TRIGGER SENSOR
		TIM3->CR1 |= TIM_CR1_CEN;
		
		// WAIT FOR TIMER 4 INTERRUPT TO SIGNAL RISING EDGE, THEN CLEAR
		while((TIM4->SR && (1 << 1)) == 0);
		TIM4->SR ^= (1 << 1);
		
		// SAVE LATCHED VALUE
		old_value = TIM4->CCR1;
		
		//WAIT FOR FALLING EDGE LATCH, CLEAR INTERRUPT
		while((TIM4->SR && (1 << 1)) == 0);
		TIM4->SR ^= (1 << 1);
		
		// SAVE LATCHED VALUE
		new_value = TIM4->CCR1;
		
		// DETERMINE DISTANCE IN INCHES
		distance = (new_value - old_value) / 158;
		
		// SEND DISTANCE OVER UART
		sprintf(print, "Distance is % 3.3fin\r\n", distance);
	
		USART_Write(USART1, print, 23);
	
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
	
	
}

void tim_4_configure(void) {
	
	// ENABLE TIMER 4 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// SELECT CHANNEL 1 AS INPUT
	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM4->CCMR1 |= (1 << 0);
	
	// SELECT NO INPUT FILTERING
	TIM4->CCMR1 &= ~TIM_CCMR1_IC1F;
	
	// SELECT CAPTURE ON RISING AND FALLING EDGE
	TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM4->CCER |= (1 << 1 | 1 << 3);
	
	// ENABLE CAPTURE FOR CHANNEL 1
	TIM4->CCER |= TIM_CCER_CC1E;
	
	// ENABLE CAPTURE INTERRUPT
	TIM4->DIER |= TIM_DIER_CC1IE;
	
	// ENABLE COUNTER
	TIM4->CR1 |= TIM_CR1_CEN;
	
}

void tim_3_configure(void) {
	
	// ENABLE TIMER 3 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// CONFIGURE PRESCALER FOR A 1MHz TIMER
	TIM3->PSC = 16;
	
	// CONFIGURE CCOMPARE REGISTER
	TIM3->CCR1 |= 20;
	
	// CONFIGURE AUTO RELOAD REGISTER
	TIM3->ARR |= 40;
	
	// ENABLE ONE PULSE MODE
	TIM3->CR1 |= TIM_CR1_OPM;
	
}


void sensor_gpio_init(void) {
	
	//ENABLE GPIOB CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// CONGIGURE GPIOB PIN 6 AS ALTERNATE FUNCTION
	GPIOB->MODER &= ~(3 << 6*2);
	GPIOB->MODER |= (2 << 6*2);
	
	// SELECT TIMER 4 AS ALTERNATE FUNCTION
	GPIOB->AFR[0] &= ~(0xF << 6*4);
	GPIOB->AFR[0] |= (2 << 6*4);
	
	// ENABLE GPIOC CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// SET PC6 AS ALTERNATE FUNCTION
	GPIOC->MODER &= ~(3 << 6*2);
	GPIOC->MODER |= (2 << 6*2);
	
	// SET TIMER 3 AS ALTERNATE FUNCTION
	GPIOC->AFR[0] &= ~(0xF << 6*4);
	GPIOC->AFR[0] |= (2 << 6*4);
	
}



