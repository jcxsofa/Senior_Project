#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>
#include <string.h>
#include "tim_13.h"
#include "adc_1.h"
#include "adc_2.h"
#include "Motor.h"
#include "tim_5.h"
#include "tim_1.h"

void sysclk_Configure(void);
void io_Configure(void);
void adc_Configure(void);
void dac_Configure(void);
void timer4_Configure(void);

#define BufferSize 32
void dac_Configure(void);
void USART_Init(USART_TypeDef * USARTx);
void gpio_Configure(void);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);
void decode(uint8_t * buffer);

uint8_t USART1_Buffer_Rx[BufferSize];// = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART1_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, '\n',};
uint8_t Rx1_Counter = 0;

int newspeed;
struct Motor M1;

int main(void)
{
		
	int transmit;
	
	
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
	
	Motor_init(&M1, .265, 151, 11.4, 1.379, 1);
	
	adc_1_gpio_init();
	adc_1_config();
	
	tim_5_gpio_init();
	tim_5_config();
	
	tim_1_gpio_init();
	tim_1_config();
	
	// INITIALIZE USART GPIO PINS
	gpio_Configure();
	
	// INITIALIZE USART1
	USART_Init(USART1);
	
	// SET PRIORITY AND ENABLE INTERRUPT
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	
	M1.Desired_Speed = 84;
	
	//dac_Configure();
	
//	adc_2_gpio_init();
//	adc_2_config();
	
	//tim_13_config();
	
	
	while(1){
		//Motor_1_ISR(&M1);
		//transmit = M1.duty_cycle * 100;
		//sprintf(USART1_Buffer_Tx, "%d", transmit);
		//USART1_Buffer_Tx[4] = '\n';
		USART_Write(USART1, USART1_Buffer_Tx, 7);
	}	
}

void sysclk_Configure(void){
	
	// ENABLE HSI
	RCC->CR |= RCC_CR_HSION;
	
	// WAIT FOR HSI
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

//	// SELECT PLL SOURCE AS HSI
//	RCC->PLLCFGR |= (1 << 22);
//	
//	// SET PLL DIVISION FACTOR M
//	RCC->PLLCFGR &= ~(0x1F);
//	RCC->PLLCFGR |= 16;
//	
//	// SET PLL MULIPLICATION FACTOR N
//	RCC->PLLCFGR &= ~(0x1FF << 6);
//	RCC->PLLCFGR |= (192 << 6);
//	
//	// SET PLL DIVISION FACTOR P
//	RCC->PLLCFGR &= ~(3 << 16);
	
//	// ENABLE PLL CLOCK
//	RCC->CR |= RCC_CR_PLLON;
//	
//	// WAIT FOR PLL CLOCK
//	while ((RCC->CR & RCC_CR_PLLRDY) == 0);
//	
//	// SELECT PLL AS SYSTEM CLOCK
//	RCC->CFGR &= ~RCC_CFGR_SW;
//	RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	// WAIT FOR HSI TO BE SELECTED
	while ((RCC->CFGR & RCC_CFGR_SWS) == !RCC_CFGR_SWS_HSI);
	
}

void TIM8_UP_TIM13_IRQHandler (void) {
	
		Motor_1_ISR(&M1);
	
	//if (TIM4->SR && TIM_SR_UIF)
	TIM13->SR ^= TIM_SR_UIF;

}




void gpio_Configure(void) {
	
	// ENABLE GPIOA CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	// ENABLE USART1 CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
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
	if( Rx1_Counter == 7 ) {
		for (i=0; i<=7; i++) {
			USART1_Buffer_Tx[i] = buffer[i];
		}
		USART_Write(USART1, USART1_Buffer_Tx, 7);
	}
	
}

void USART1_IRQHandler(void) {
	USART_IRQHandler(USART1, USART1_Buffer_Rx, &Rx1_Counter);
	//decode(USART1_Buffer_Rx);
	//USART_Write(USART1, USART1_Buffer_Tx, 7);
}

void dac_Configure(void) {
	
	// ENABLE DAC CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	// ENABLE GPIOC CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	// ENABLE DAC TRIGGERING
	DAC->CR= DAC_CR_TEN1;
	
	// SELECT SOFTWARE TRIGGER FOR DAC CONVERSION
	DAC->CR |= DAC_CR_TSEL1;
	
	// CONFIGURE PC0 AS ANALOG
	GPIOC->MODER |= GPIO_MODER_MODER0;
	
	// ENABLE DAC CONVESION
	DAC->CR |= DAC_CR_EN1;
	
}