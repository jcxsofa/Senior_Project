#include "adc_gpio.h"

void adc_gpio_init(void) {
	
		/* CONFIGURE INPUT PINS */
	/*
		PORT A PINS 4, 5, 6, and 7, AND PORT C
		PINS 0, 1, 2, AND 3 ARE USED FOR ADC 
		CHANNELS 4, 5, 6, and 7, AND 10, 11, 12,
		AND 13 RESPECTIVELY. THEY ARE CONFIGURED AS
		ANALOG MODE, NO PULL-UP/PULL-DOWN.
	*/
	
	// ENABLE PORT A CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	// ENABLE PORT C CLOCK
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	/* SET AS ANALOG */
	
	// PA4
	GPIOA->MODER |= (3 << 4*2);
	
	// PA5
	GPIOA->MODER |= (3 << 5*2);
	
	// PA6
	GPIOA->MODER |= (3 << 6*2);
	
	// PA7
	GPIOA->MODER |= (3 << 7*2);
	
	// PC0
	GPIOC->MODER |= (3 << 0*2);
	
	// PC1
	GPIOC->MODER |= (3 << 1*2);
	
	// PC2
	GPIOC->MODER |= (3 << 2*2);
	
	// PC3
	GPIOC->MODER |= (3 << 3*2);
	
	/* SET AS NO PULL-UP/PULL-DOWN */
	
	// PA4
	GPIOA->PUPDR &= ~(3 << 4*2);
	
	// PA5
	GPIOA->PUPDR &= ~(3 << 5*2);
	
	// PA6
	GPIOA->PUPDR &= ~(3 << 6*2);
	
	// PA7
	GPIOA->PUPDR &= ~(3 << 7*2);
	
	// PC0
	GPIOC->PUPDR &= ~(3 << 0*2);
	
	// PC1
	GPIOC->PUPDR &= ~(3 << 1*2);
	
	// PC2
	GPIOC->PUPDR &= ~(3 << 2*2);
	
	// PC3
	GPIOC->PUPDR &= ~(3 << 3*2);
	
}
