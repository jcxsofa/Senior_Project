#define ARM_MATH_CM4
#include <stdint.h>
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>
#include "tim_13.h"
#include "adc_1.h"
#include "adc_2.h"
#include "Motor.h"

void sysclk_Configure(void);
void io_Configure(void);
void adc_Configure(void);
void dac_Configure(void);
void timer4_Configure(void);

#define blocksize 1

struct Motor M1;

int main(void)
{
		
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
	
	Motor_init(&M1, .4, 120, 11.4, 1.6);
	
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
	
	float32_t test_inputf[blocksize];
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

	arm_biquad_cascade_df2T_f32(&M1.filter, test_inputf, test_outputf, blocksize);
	
	for (x = 0; x < blocksize; x++) {
		sum += test_outputf[x];
	}
	
	sum /= blocksize;
	
	sum = (sum * 3) / 0xFFF;
	
	//if (TIM4->SR && TIM_SR_UIF)
	TIM13->SR ^= TIM_SR_UIF;

}
