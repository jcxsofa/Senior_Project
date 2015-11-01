#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>
#include "tim_13.h"
#include "adc_1.h"
#include "adc_2.h"

void sysclk_Configure(void);
void io_Configure(void);
void adc_Configure(void);
void dac_Configure(void);
void timer4_Configure(void);

//#define ntaps 151
#define blocksize 1

//float32_t fcoef[ntaps] = {0.000002, 0.000004, 0.000007, 0.000011, 0.000016, 0.000022, 0.000030, 0.000040, 0.000050, 0.000062, 0.000075, 0.000089, 0.000102, 0.000114, 0.000125, 0.000133, 0.000136, 0.000134, 0.000124, 0.000105, 0.000076, 0.000034, -0.000022, -0.000094, -0.000183, -0.000290, -0.000416, -0.000560, -0.000723, -0.000903, -0.001099, -0.001307, -0.001525, -0.001748, -0.001970, -0.002185, -0.002386, -0.002565, -0.002713, -0.002821, -0.002878, -0.002875, -0.002801, -0.002646, -0.002399, -0.002050, -0.001593, -0.001017, -0.000318, 0.000510, 0.001471, 0.002564, 0.003790, 0.005144, 0.006622, 0.008216, 0.009915, 0.011707, 0.013576, 0.015506, 0.017479, 0.019473, 0.021467, 0.023438, 0.025363, 0.027220, 0.028983, 0.030631, 0.032143, 0.033498, 0.034677, 0.035666, 0.036449, 0.037017, 0.037360, 0.037476, 0.037360, 0.037017, 0.036449, 0.035666, 0.034677, 0.033498, 0.032143, 0.030631, 0.028983, 0.027220, 0.025363, 0.023438, 0.021467, 0.019473, 0.017479, 0.015506, 0.013576, 0.011707, 0.009915, 0.008216, 0.006622, 0.005144, 0.003790, 0.002564, 0.001471, 0.000510, -0.000318, -0.001017, -0.001593, -0.002050, -0.002399, -0.002646, -0.002801, -0.002875, -0.002878, -0.002821, -0.002713, -0.002565, -0.002386, -0.002185, -0.001970, -0.001748, -0.001525, -0.001307, -0.001099, -0.000903, -0.000723, -0.000560, -0.000416, -0.000290, -0.000183, -0.000094, -0.000022, 0.000034, 0.000076, 0.000105, 0.000124, 0.000134, 0.000136, 0.000133, 0.000125, 0.000114, 0.000102, 0.000089, 0.000075, 0.000062, 0.000050, 0.000040, 0.000030, 0.000022, 0.000016, 0.000011, 0.000007, 0.000004, 0.000002};
//float32_t pstate[ntaps + blocksize - 1];	
//float32_t input[blocksize];
//float32_t output[blocksize];
//int count = 0;
//arm_fir_instance_f32 filter;

#define numStages 2

float32_t standard_coeffs [5 * numStages] = {
	0.0001,   -0.0002,    0.0001,   -1.9599,    0.9605,
    1.0000,   -1.9628,    1.0000,   -1.9826,    0.9839};
//q31_t pCoeffs [5 * 2];
//q63_t pState [4 * 2];
//uint8_t postShift = 0;
//arm_biquad_cas_df1_32x64_ins_q31 filter;
float32_t pState[2*numStages];
arm_biquad_cascade_df2T_instance_f32 filter;

int main(void)
{
		
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();	
	
	//arm_fir_init_f32(&filter, ntaps, (float32_t *)&fcoef[0], &pstate[0], blocksize);
	//arm_float_to_q31(standard_coeffs, pCoeffs, 5 * 2);
	//arm_biquad_cas_df1_32x64_init_q31(&filter, 2, pCoeffs, pState, 0);
	arm_biquad_cascade_df2T_init_f32(&filter, numStages, standard_coeffs, pState);
	
	adc_1_gpio_init();
	adc_1_config();
	
//	adc_2_gpio_init();
//	adc_2_config();
	
	tim_13_config();
	
	
	while(1){
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

void TIM8_UP_TIM13_IRQHandler (void) {
	
	int test_input[blocksize];
	float32_t test_inputf[blocksize];
	//q31_t test_in [blocksize];
	//q31_t test_out [blocksize];
	float32_t test_outputf[blocksize];
	int x;
	float32_t sum = 0;
		
		
	for (x = 0; x < blocksize; x++) {

		// BEGIN CONVERSIONS
		ADC1->CR2 |= ADC_CR2_SWSTART;
		
		while ((ADC1->SR & ADC_SR_EOC) == 0)

		// RESET INTERRUPT ENABLE
		ADC1->CR1 |= ADC_CR1_EOCIE;
		
		// CLEAR EOC AGAIN?
		ADC1->SR &= ~ADC_SR_EOC;
		
		test_inputf[x] = ADC1->DR;

	}
	
	//arm_fir_f32(&filter, test_input, test_output, blocksize); 
	//arm_float_to_q31(test_input, test_in, blocksize);
	//arm_biquad_cas_df1_32x64_q31(&filter, test_in, test_out, blocksize);
	arm_biquad_cascade_df2T_f32(&filter, test_inputf, test_outputf, blocksize);
	//arm_q31_to_float(test_out, test_output, blocksize);
	for (x = 0; x < blocksize; x++) {
		sum += test_outputf[x];
	}
	
	sum /= blocksize;
	
	sum = (sum * 3) / 0xFFF;
	
	//if (TIM4->SR && TIM_SR_UIF)
	TIM13->SR ^= TIM_SR_UIF;

}
