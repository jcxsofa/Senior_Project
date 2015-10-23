#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>

void sysclk_Configure(void);
void io_Configure(void);
void adc_Configure(void);
void dac_Configure(void);
void timer4_Configure(void);

#define ntaps 101
#define blocksize 10

float32_t fcoef[ntaps] = {-0.000002, -0.000006, -0.000012, -0.000022, -0.000037, -0.000057, -0.000085, -0.000121, -0.000167, -0.000222, -0.000286, -0.000359, -0.000440, -0.000526, -0.000612, -0.000695, -0.000767, -0.000821, -0.000846, -0.000834, -0.000771, -0.000645, -0.000442, -0.000147, 0.000252, 0.000770, 0.001419, 0.002210, 0.003152, 0.004251, 0.005510, 0.006929, 0.008502, 0.010222, 0.012074, 0.014042, 0.016105, 0.018236, 0.020407, 0.022586, 0.024739, 0.026829, 0.028823, 0.030683, 0.032375, 0.033867, 0.035131, 0.036141, 0.036877, 0.037325, 0.037476, 0.037325, 0.036877, 0.036141, 0.035131, 0.033867, 0.032375, 0.030683, 0.028823, 0.026829, 0.024739, 0.022586, 0.020407, 0.018236, 0.016105, 0.014042, 0.012074, 0.010222, 0.008502, 0.006929, 0.005510, 0.004251, 0.003152, 0.002210, 0.001419, 0.000770, 0.000252, -0.000147, -0.000442, -0.000645, -0.000771, -0.000834, -0.000846, -0.000821, -0.000767, -0.000695, -0.000612, -0.000526, -0.000440, -0.000359, -0.000286, -0.000222, -0.000167, -0.000121, -0.000085, -0.000057, -0.000037, -0.000022, -0.000012, -0.000006, -0.000002};
float32_t pstate[ntaps + blocksize - 1];	
float32_t input[blocksize];
float32_t output[blocksize];
int count = 0;
arm_fir_instance_f32 filter;
	
int main(void)
{
		
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();	
	
	/* IO CONFIGURE */
	io_Configure();
	
	/* DAC CONFIGURE */
	dac_Configure();
	
	/* ADC CONFIGURE */
	adc_Configure();
	
	/* TIMER 4 CONFIGURE */
	timer4_Configure();
	
	arm_fir_init_f32(&filter, ntaps, (float32_t *)&fcoef[0], &pstate[0], blocksize);
	
	while(1){
	}	
}

void TIM4_IRQHandler (void) {
		int x;
	


		if (count == blocksize) {
			arm_fir_f32(&filter, input, output, 10);
//			for (count = 0; count < 10; count++)
//				output[count] = input[count];
			count = 0;
		}
			
			
	
		// BEGIN CONVERSIONS
		ADC1->CR2 |= ADC_CR2_SWSTART;
		
		while ((ADC1->SR & ADC_SR_EOC) == 0)
	
		// CLEAR EOC BIT
		//ADC1->SR &= ~ADC_SR_EOC;
		// RESET INTERRUPT ENABLE
		ADC1->CR1 |= ADC_CR1_EOCIE;
		// CLEAR EOC AGAIN?
		ADC1->SR &= ~ADC_SR_EOC;
		
		input[count] = ADC1->DR;

		x = output[count];
		
		// COPY DATA TO DAC OUTPUT
		DAC->DHR12R1 &= 0xFFFFF000;
		DAC->DHR12R1 |= x;
		
		// BEGIN DAC CONVERSION
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
		
		count++;
		
		//if (TIM4->SR && TIM_SR_UIF)
		TIM4->SR ^= TIM_SR_UIF;
}


//void ADC_IRQHandler (void){
//	int x;
//	// CHECK FOR END OF CONVERSION
//	if ((ADC1->SR & ADC_SR_EOC) != 0){
//		// CLEAR EOC BIT
//		ADC1->SR &= ~ADC_SR_EOC;
//		// RESET INTERRUPT ENABLE
//		ADC1->CR1 |= ADC_CR1_EOCIE;
//		// CLEAR EOC AGAIN?
//		ADC1->SR &= ~ADC_SR_EOC;

//		buffer[0] = ADC1->DR;
//		
//		arm_fir_f32(filter, buffer, buffer, 1);

//		x = buffer[0];
//		
//		// COPY DATA TO DAC OUTPUT
//		DAC->DHR12R1 &= 0xFFFFF000;
//		DAC->DHR12R1 |= x;
//		
//		// BEGIN DAC CONVERSION
//		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
//		
//	}
//	// START CONVERSIONS AGAIN
//	ADC1->CR2 |= ADC_CR2_SWSTART;
//			
//}

void sysclk_Configure(void){
	
	// ENABLE HSI
	RCC->CR |= RCC_CR_HSION;
	
	// WAIT FOR HSI
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

	// SELECT HSI AS SYSTEM CLOCK
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_HSI;
	
	// WAIT FOR HSI TO BE SELECTED
	while ((RCC->CFGR & RCC_CFGR_SWS) == !RCC_CFGR_SWS_HSI);
	
}


	/* I/O CONFIGURATION */
void io_Configure(void) {
	// ENABLE GPIO A CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// ENABLE GPIO B CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// ENABLE GPIO C CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	// ENABLE TIMER 4 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// CONFIGURE PA4 AND PA5 AS ANALOG MODE
	GPIOA->MODER |= GPIO_MODER_MODER4;
	GPIOA->MODER |= GPIO_MODER_MODER5;
	
	// CONFIGURE PC0 AS ANALOG
	GPIOC->MODER |= GPIO_MODER_MODER0;
}


	/* ADC CONFIGURATION */
void adc_Configure(void){
	// ENABLE ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// DISABLE CONVERSION
	ADC1->CR2 &= ~ADC_CR2_ADON;
	
	// CONFIGURE REGULAR CHANNEL SEQUENCE LENGTH TO 1 CONVERSION
	ADC1->SQR1 &= ~ADC_SQR1_L;
	
	// SELECT CHANNEL 10 AS FIRST AND ONLY CONVERSION
	ADC1->SQR3 |= 0xA;
	
	// SELECT 16 CYCLES PER SAMPLE FOR CONVERSION
	ADC1->SMPR1 |= 0x2;
	
////	// ENABLE ANALOG WATCHDOG
////	ADC1->CR1 |- ADC_CR1_AWDEN;
////	
////	// ENABLE ANALOG WATCHDOG INTERRUPT
////	ADC1->CR1 |= ADC_CR1_AWDIE;
////	
////	// SELECT CHANNEL 10 FOR WATCH DOG
////	ADC1->CR1 |= 0xA;

	// ENABLE END OF CONVERSION INTERRUPT
	ADC1->CR1 |= ADC_CR1_EOCIE;
	
	// ENABLE CONTINUOUS CONVERSION
	//ADC1->CR2 |= ADC_CR2_CONT;
	
//	// ENABLE ADC INTERRUPT AND SET PRIORITY
//	NVIC_SetPriority(ADC_IRQn, 0);
//	NVIC_EnableIRQ(ADC_IRQn);
	
	// ENABLE ADC
	ADC1->CR2 |= ADC_CR2_ADON;
	
	
}

void dac_Configure(void) {
	
	// ENABLE DAC CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	// ENABLE DAC TRIGGERING
	DAC->CR= DAC_CR_TEN1;
	
	// SELECT SOFTWARE TRIGGER FOR DAC CONVERSION
	DAC->CR |= DAC_CR_TSEL1;
	
	// ENABLE DAC CONVESION
	DAC->CR |= DAC_CR_EN1;
	
}

	/* TIMER 4 CONFIGURATION */
void timer4_Configure(void){
	
	// ENABLE TIMER 4 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// TIMER 4 PRESCALER BASED OFF 16MHZ HSI
	TIM4->PSC |= 0;
	
	// AUTO RELOAD REGISTER
	TIM4->ARR &= ~TIM_ARR_ARR;
	TIM4->ARR |= 533;
	
	// ENABLE AUTO RELAOD
	TIM4->CR1 |= TIM_CR1_ARPE;
	
	// ENABLE UPDATE INTERRUPT
	TIM4->DIER |= TIM_DIER_UIE;
	
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	
	//ENABLE TIMER
	TIM4->CR1 |= TIM_CR1_CEN;
	
}	
