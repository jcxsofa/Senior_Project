#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>
#include <string.h>
#include "Motor.h"
#include "adc_gpio.h"
#include "adc_1.h"
#include "tim_1.h"
#include "tim_2.h"
#include "tim_3.h"
#include "tim_4.h"
#include "tim_5.h"
#include "tim_9.h"
#include "tim_10.h"
#include "tim_12.h"
#include "tim_13.h"
#include "uart.h"

void sysclk_Configure(void);
void decode(uint8_t * buffer);
void calc_error(void);

uint8_t USART2_Buffer_Rx[BufferSize];// = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67};
uint8_t USART2_Buffer_Tx[BufferSize] = {0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, '\n',};
uint8_t Rx2_Counter = 0;

#define BufferSize 32
void dac_Configure(void);

struct Motor M1, M2, M3, M4;

int main(void)
{
		
	int transmit;
	
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
	
	Motor_init(&M1, .26, 151, 11.4, 1.656, 13.014, 1);
	Motor_init(&M2, .25, 145, 11.4, 1.723, 12.44, 2);
	Motor_init(&M3, .24, 144, 11.4, 1.879, 12.43, 3);
	Motor_init(&M4, .3, 152, 11.4, 1.471, 13.12, 4);
	
	adc_gpio_init();
	adc_1_config();
	
	tim_9_gpio_init();
	tim_9_config();
	
	tim_12_gpio_init();
	tim_12_config();
	
	uart_gpio_init();
	USART_Init(USART2);
	
	tim_10_config();
	tim_13_config();
	
	tim_5_gpio_init();
	tim_5_config();
	
	tim_1_gpio_init();
	tim_1_config();
	
	tim_2_gpio_init();
	tim_2_config();
	
	tim_3_gpio_init();
	tim_3_config();
	
	tim_4_gpio_init();
	tim_4_config();
	
	M1.Desired_Speed = 0;
	M2.Desired_Speed = 0;
	M3.Desired_Speed = 0;
	M4.Desired_Speed = 0;
		
	
	while(1){
//		Motor_ISR(&M1);
//		Motor_ISR(&M2);
//		Motor_ISR(&M3);
//		Motor_ISR(&M4);
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
	char print[15];
	float average;
	// SERVICE DATA UPDATE
	if (TIM10->SR && TIM_SR_UIF) {
		
		// CLEAR TERMINAL
		strcpy(print, "\033[2J\033[1;1H");
		USART_Write(USART2, print, 10);		
		
		// CALCULATE ERROR
		calc_error();
		
		// DISPLAY STUFF
		display_stats(&M1);
		display_stats(&M2);
		display_stats(&M3);
		display_stats(&M4);	
		
		average = (M1.Encoder_Speed + M2.Encoder_Speed + M3.Encoder_Speed + M4.Encoder_Speed) / 4;
	
		sprintf(print, "Average speed = % 3.3frpm\n\r", average);
		
		USART_Write(USART2, print, 28);
	}
	
	// RESET INTERRUPT
	TIM10->SR ^= TIM_SR_UIF;
}


void decode(uint8_t * buffer) {
	int i = 0;
	char instruction[10];
	char stop[10];
	char move[10];
	char back[10];
	strcpy(stop, "stop");
	strcpy(move, "move");
	strcpy(back, "back");
	if( Rx2_Counter == 4 ) {
		
		strncpy(instruction, buffer, 4);
		i = strcmp(instruction, stop);
		if( i == 0) {
			M1.Desired_Speed = 0;
			M2.Desired_Speed = 0;
			M3.Desired_Speed = 0;
			M4.Desired_Speed = 0;
		}
		i = strcmp(instruction, move);
		if( i == 0) {
			M1.Desired_Speed = 84;
			M2.Desired_Speed = 84;
			M3.Desired_Speed = 84;
			M4.Desired_Speed = 84;
		}
		
		i = strcmp(instruction, back);
		if( i == 0) {
			M1.Desired_Speed = -84;
			M2.Desired_Speed = -84;
			M3.Desired_Speed = -84;
			M4.Desired_Speed = -84;
		}
	}
	
}

void calc_error(void) {
	
	double newencoder = 0;
	double test_val;
	
	newencoder = TIM1->CNT;
	test_val = (newencoder - M1.OldEncoder);
	M1.OldEncoder = newencoder;
	M1.Encoder_Speed = ((test_val/2.02)/280)*60;
	M1.Error = 100 * ((M1.Encoder_Speed - M1.Desired_Speed) / M1.Desired_Speed);
	
	newencoder = TIM2->CNT;
	test_val = (newencoder - M2.OldEncoder);
	M2.OldEncoder = newencoder;
	M2.Encoder_Speed = ((test_val/2.02)/280)*60;
	M2.Error = 100 * ((M2.Encoder_Speed - M2.Desired_Speed) / M2.Desired_Speed);
	
	newencoder = TIM4->CNT;
	test_val = (newencoder - M3.OldEncoder);
	M3.OldEncoder = newencoder;
	M3.Encoder_Speed = ((test_val/1.02)/280)*60;
	M3.Error = 100 * ((M3.Encoder_Speed - M3.Desired_Speed) / M3.Desired_Speed);
	
	newencoder = TIM3->CNT;
	test_val = (newencoder - M4.OldEncoder);
	M4.OldEncoder = newencoder;
	M4.Encoder_Speed = ((test_val/1.02)/280)*60;
	M4.Error = 100 * ((M4.Encoder_Speed - M4.Desired_Speed) / M4.Desired_Speed);
	
	
	
	
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