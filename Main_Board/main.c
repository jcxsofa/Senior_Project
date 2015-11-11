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
#include "tim_9.h"
#include "tim_12.h"
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
struct Motor M1;

int main(void)
{
		
	int transmit;
	char result[10];
	char print[26];
	
	
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
	
	Motor_init(&M1, .265, 151, 11.4, 1.379, 1);
	
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
	
	// SET PRIORITY AND ENABLE INTERRUPT
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	
	M1.Desired_Speed = -80;
	//dac_Configure();
	
//	adc_2_gpio_init();
//	adc_2_config();
	
	//tim_13_config();
	
	
	while(1){
		Motor_ISR(&M1);
		
		sprintf(result, "% 3.3f  ", M1.BEMF_Speed);
		
		strcpy(print, "Motor 1 Speed = ");
		strcat(print, result);
		
		USART_Write(USART2, print, 26);
		
		sprintf(result, "% 3.3f\n\r", M1.duty_cycle);
		
		strcpy(print, "Motor 1 Speed = ");
		strcat(print, result);
		
		USART_Write(USART2, print, 26);
	
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
		
		Motor_ISR(&M1);
	
	//if (TIM4->SR && TIM_SR_UIF)
	TIM13->SR ^= TIM_SR_UIF;

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