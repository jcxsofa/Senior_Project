#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>
#include "tim_5.h"
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