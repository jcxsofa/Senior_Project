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

float32_t enc_coef[ntaps] = {0.000026, 0.000028, 0.000031, 0.000034, 0.000036, 0.000039, 0.000042, 0.000044, 0.000047, 0.000049, 0.000052, 0.000054, 0.000056, 0.000057, 0.000059, 0.000060, 0.000060, 0.000061, 0.000061, 0.000060, 0.000059, 0.000057, 0.000055, 0.000052, 0.000049, 0.000045, 0.000040, 0.000034, 0.000028, 0.000020, 0.000012, 0.000003, -0.000007, -0.000019, -0.000031, -0.000044, -0.000058, -0.000074, -0.000090, -0.000108, -0.000127, -0.000147, -0.000168, -0.000190, -0.000213, -0.000237, -0.000263, -0.000290, -0.000317, -0.000346, -0.000375, -0.000406, -0.000437, -0.000469, -0.000502, -0.000535, -0.000569, -0.000604, -0.000639, -0.000674, -0.000709, -0.000744, -0.000780, -0.000815, -0.000850, -0.000884, -0.000918, -0.000951, -0.000983, -0.001014, -0.001043, -0.001071, -0.001098, -0.001123, -0.001146, -0.001166, -0.001185, -0.001201, -0.001214, -0.001224, -0.001231, -0.001234, -0.001234, -0.001230, -0.001223, -0.001211, -0.001195, -0.001174, -0.001149, -0.001119, -0.001083, -0.001043, -0.000996, -0.000945, -0.000887, -0.000824, -0.000755, -0.000680, -0.000598, -0.000510, -0.000415, -0.000314, -0.000206, -0.000092, 0.000029, 0.000157, 0.000292, 0.000433, 0.000582, 0.000737, 0.000899, 0.001067, 0.001242, 0.001424, 0.001612, 0.001807, 0.002007, 0.002214, 0.002427, 0.002645, 0.002869, 0.003098, 0.003333, 0.003572, 0.003816, 0.004064, 0.004316, 0.004572, 0.004831, 0.005094, 0.005359, 0.005627, 0.005896, 0.006168, 0.006441, 0.006715, 0.006989, 0.007263, 0.007538, 0.007811, 0.008084, 0.008355, 0.008624, 0.008891, 0.009155, 0.009416, 0.009673, 0.009927, 0.010175, 0.010419, 0.010658, 0.010891, 0.011118, 0.011338, 0.011552, 0.011758, 0.011957, 0.012148, 0.012330, 0.012504, 0.012669, 0.012825, 0.012972, 0.013108, 0.013235, 0.013351, 0.013457, 0.013553, 0.013637, 0.013711, 0.013773, 0.013824, 0.013864, 0.013893, 0.013910, 0.013916, 0.013910, 0.013893, 0.013864, 0.013824, 0.013773, 0.013711, 0.013637, 0.013553, 0.013457, 0.013351, 0.013235, 0.013108, 0.012972, 0.012825, 0.012669, 0.012504, 0.012330, 0.012148, 0.011957, 0.011758, 0.011552, 0.011338, 0.011118, 0.010891, 0.010658, 0.010419, 0.010175, 0.009927, 0.009673, 0.009416, 0.009155, 0.008891, 0.008624, 0.008355, 0.008084, 0.007811, 0.007538, 0.007263, 0.006989, 0.006715, 0.006441, 0.006168, 0.005896, 0.005627, 0.005359, 0.005094, 0.004831, 0.004572, 0.004316, 0.004064, 0.003816, 0.003572, 0.003333, 0.003098, 0.002869, 0.002645, 0.002427, 0.002214, 0.002007, 0.001807, 0.001612, 0.001424, 0.001242, 0.001067, 0.000899, 0.000737, 0.000582, 0.000433, 0.000292, 0.000157, 0.000029, -0.000092, -0.000206, -0.000314, -0.000415, -0.000510, -0.000598, -0.000680, -0.000755, -0.000824, -0.000887, -0.000945, -0.000996, -0.001043, -0.001083, -0.001119, -0.001149, -0.001174, -0.001195, -0.001211, -0.001223, -0.001230, -0.001234, -0.001234, -0.001231, -0.001224, -0.001214, -0.001201, -0.001185, -0.001166, -0.001146, -0.001123, -0.001098, -0.001071, -0.001043, -0.001014, -0.000983, -0.000951, -0.000918, -0.000884, -0.000850, -0.000815, -0.000780, -0.000744, -0.000709, -0.000674, -0.000639, -0.000604, -0.000569, -0.000535, -0.000502, -0.000469, -0.000437, -0.000406, -0.000375, -0.000346, -0.000317, -0.000290, -0.000263, -0.000237, -0.000213, -0.000190, -0.000168, -0.000147, -0.000127, -0.000108, -0.000090, -0.000074, -0.000058, -0.000044, -0.000031, -0.000019, -0.000007, 0.000003, 0.000012, 0.000020, 0.000028, 0.000034, 0.000040, 0.000045, 0.000049, 0.000052, 0.000055, 0.000057, 0.000059, 0.000060, 0.000061, 0.000061, 0.000060, 0.000060, 0.000059, 0.000057, 0.000056, 0.000054, 0.000052, 0.000049, 0.000047, 0.000044, 0.000042, 0.000039, 0.000036, 0.000034, 0.000031, 0.000028, 0.000026};

	// Low Pass FIR Filter
	arm_fir_instance_f32 enc_filter;

#define BufferSize 32
void dac_Configure(void);


double oldencoder = 0;
double newencoder = 0;
struct Motor M1, M2, M3, M4;

int main(void)
{
		
	int transmit;
	// Low Pass FIR Filter
	arm_fir_instance_f32 enc_filter;
	// STATE BUFFER
	float32_t enc_state[ntaps + blocksize - 1];
	
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
	
	arm_fir_init_f32(&enc_filter, ntaps, (float32_t *)&enc_coef[0], &enc_state[0], blocksize);
	
	Motor_init(&M1, .265, 151, 11.4, 1.379, 1);
	Motor_init(&M2, .265, 151, 11.4, 1.379, 2);
	Motor_init(&M3, .265, 151, 11.4, 1.379, 3);
	Motor_init(&M4, .265, 151, 11.4, 1.379, 4);
	
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
	
	M1.Desired_Speed = 80;
	M2.Desired_Speed = 60;
	M3.Desired_Speed = 60;
	M4.Desired_Speed = 60;
		
	
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
	char clear[10];
	float test_val;
	// SERVICE DATA UPDATE
	if (TIM10->SR && TIM_SR_UIF) {
		
		// CLEAR TERMINAL
		strcpy(clear, "\033[2J\033[1;1H");
		USART_Write(USART2, clear, 10);
		
		newencoder = TIM1->CNT;
		
		test_val = (newencoder - oldencoder);
		oldencoder = newencoder;
		
		arm_fir_f32(&enc_filter, &test_val, &test_val, 1);
		
		M1.Encoder_Speed = test_val;
		
		// DISPLAY STUFF
		display_speed_current(&M1);
//		display_speed_current(&M2);
//		display_speed_current(&M3);
//		display_speed_current(&M4);
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