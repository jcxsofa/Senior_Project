#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>
#include <string.h>
#include "Motor.h"
#include "adc_1.h"
#include "adc_2.h"
#include "tim_5.h"
#include "tim_9.h"
#include "tim_10.h"
#include "tim_12.h"
#include "tim_13.h"
#include "uart.h"

void sysclk_Configure(void);
void decode(uint8_t * buffer);

uint8_t USART2_Buffer_Rx[BufferSize];// = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART2_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, '\n',};
uint8_t Rx2_Counter = 0;

#define BufferSize 32
void dac_Configure(void);


double oldencoder = 0;
double newencoder = 0;
struct Motor M1, M2, M3, M4;

int main(void)
{
		
	int transmit;
	
	
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
	
	Motor_init(&M1, .265, 151, 11.4, 1.379, 1);
	Motor_init(&M2, .265, 151, 11.4, 1.379, 2);
	Motor_init(&M3, .265, 151, 11.4, 1.379, 3);
	Motor_init(&M4, .265, 151, 11.4, 1.379, 4);
	
	adc_1_gpio_init();
	adc_1_config();
	
	adc_2_gpio_init();
	adc_2_config();
	
	tim_5_gpio_init();
	tim_5_config();
	
	tim_9_gpio_init();
	tim_9_config();
	
	tim_12_gpio_init();
	tim_12_config();
	
	uart_gpio_init();
	USART_Init(USART2);
	
	tim_10_config();
	tim_13_config();
	
	M1.Desired_Speed = 20;
	M2.Desired_Speed = -30;
	M3.Desired_Speed = 40;
	M4.Desired_Speed = -50;
		
	
	while(1){
		Motor_ISR(&M1);
		Motor_ISR(&M2);
		Motor_ISR(&M3);
		Motor_ISR(&M4);
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
	
	// SERVICE MOTOR CONTROL INTERRUPT
	if (TIM13->SR && TIM_SR_UIF) {
		Motor_ISR(&M1);
		Motor_ISR(&M2);
		Motor_ISR(&M3);
		Motor_ISR(&M4);
	}
	
	// RESET INTERRUPT
	TIM13->SR ^= TIM_SR_UIF;
}

void TIM1_UP_TIM10_IRQHandler (void) {
	char clear[10];
	// SERVICE DATA UPDATE
	if (TIM10->SR && TIM_SR_UIF) {
		
		// CLEAR TERMINAL
		strcpy(clear, "\033[2J\033[1;1H");
		USART_Write(USART2, clear, 10);
		
		// DISPLAY STUFF
		display_speed_current(&M1);
		display_speed_current(&M2);
		display_speed_current(&M3);
		display_speed_current(&M4);
	}
	
	// RESET INTERRUPT
	TIM10->SR ^= TIM_SR_UIF;
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