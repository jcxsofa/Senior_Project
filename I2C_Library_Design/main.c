#include <stdint.h>
#include "stm32l1xx.h"

void sysclk_Configure(void);
void io_Configure(void);
void pb_Configure(void);
void timer4_Configure(void);
void i2c2_configuration(void);
void i2c2_send_byte(unsigned char byte);
void i2c1_configuration(void);
void i2c1_send_byte(unsigned char byte);
unsigned char reverse_char(unsigned char byte);
void delay_uS(int delay);

int main(void)
{
	int i;
	
	/* SYSTEM CLOCK CONFIGURE */
	sysclk_Configure();
		
	/* I/O CONFIGURATION */
	io_Configure();
	
	/* PUSH BUTTON INTERRUPT CONFIGURE */
	pb_Configure();
	
	/* TIMER 4 CONFIGURATION */
	timer4_Configure();
	
	delay_uS(64000);
	
	/* I2C2 CONFIGURATION */
	i2c1_configuration();
	
	// DELAY TO ALLOW DISPLAY TO STABILIZE
	for (i = 0; i < 1000; i++) delay_uS(64000);
	
	while(1){
	
		// SEND COMMAND TO TURN ON DISPLAY
		i2c1_send_byte(0x0C);
		
		//for (i = 0; i < 250; i++) delay_uS(64000);
		
		// SEND COMMAND TO TURN OFF DISPLAY
		i2c1_send_byte(0x08);
		
		//for (i = 0; i < 250; i++) delay_uS(64000);
	}
}

void i2c1_send_byte(unsigned char byte) {
	
	uint8_t add = 0x27;
	int dump;
	
	// SEND START BIT
	I2C1->CR1 |= I2C_CR1_START;
	
	// WAIT FOR SB AFFIRMATION
	while((I2C1->SR1 & I2C_SR1_SB) == 0);
	
	// DO A READ FOR KICKS
	dump = I2C1->SR1;
	
	delay_uS(64000);
	
	// WRITE ADDRESS WITH LSB SET TO 0 TO INDICATE WRITE
	I2C1->DR = (add << 1);
	
	// DO A READ FOR KICKS
	dump = I2C1->SR1;
	
	// WAIT FOR ADDR
	while((I2C1->SR1 & I2C_SR1_ADDR) == 0);
	
	// READ SR2 TO CLEAR ADDR
	I2C1->SR2;
	
	// WAIT FOR TXE BIT
	while((I2C1->SR1 & I2C_SR1_TXE) == 0);
	
	// WRITE DATA BYTE
	I2C1->DR = byte;//reverse_char(byte);
	
	// WAIT FOR TXE BIT
	while((I2C1->SR1 & I2C_SR1_TXE) == 0);
	
	// WRITE STOP BIT
	I2C1->CR1 |= I2C_CR1_STOP;
	
	// WAIT FOR BUSY CLEAR
	while((I2C1->SR1 & I2C_SR2_BUSY) == 0);
	
}

void i2c2_send_byte(unsigned char byte) {
	
	uint8_t add = 0x27;
	int dump;
	
	// SEND START BIT
	I2C2->CR1 |= I2C_CR1_START;
	
	// WAIT FOR SB AFFIRMATION
	while((I2C2->SR1 & I2C_SR1_SB) == 0);
	
	// DO A READ FOR KICKS
	dump = I2C2->SR1;
	
	delay_uS(64000);
	
	// WRITE ADDRESS WITH LSB SET TO 0 TO INDICATE WRITE
	I2C2->DR = (add << 1);
	
	// DO A READ FOR KICKS
	dump = I2C2->SR1;
	
	// WAIT FOR ADDR
	while((I2C2->SR1 & I2C_SR1_ADDR) == 0);
	
	// READ SR2 TO CLEAR ADDR
	I2C2->SR2;
	
	// WAIT FOR TXE BIT
	while((I2C2->SR1 & I2C_SR1_TXE) == 0);
	
	// WRITE DATA BYTE
	I2C2->DR = byte;//reverse_char(byte);
	
	// WAIT FOR TXE BIT
	while((I2C2->SR1 & I2C_SR1_TXE) == 0);
	
	// WRITE STOP BIT
	I2C2->CR1 |= I2C_CR1_STOP;
	
	// WAIT FOR BUSY CLEAR
	while((I2C2->SR1 & I2C_SR2_BUSY) == 0);
	
}

void i2c1_configuration(void) {
	
	// RESET I2C1
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
	
	// ENABLE I2C1
	I2C1->CR1 |= I2C_CR1_PE;
	
	// DO A SOFTWARE RESET
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	
	
	/* CONFIGURE I2C1 CONTROL REGISTERS */
	
	// SET SMBBus MODE TO I2C
	I2C1->CR1 &= ~I2C_CR1_SMBUS;
	
	// CLEAR PERIPHERAL CLOCK FREQUENCY AND SET AS 2 MHz
	I2C1->CR2 &= ~I2C_CR2_FREQ;
	I2C1->CR2 |= 16;
	
	// EVENT INTERRUPT ENABLE
	I2C1->CR2 |= I2C_CR2_ITEVTEN;
	
	// DISABLE I2C TO CONFIGURE TRISE
	I2C1->CR1 &= ~I2C_CR1_PE;
	
	/* CLEAR MAX RISE TIME, SET FOR MAX
		RISE TIME OF STANDARD MODE */
	I2C1->TRISE &= ~I2C_TRISE_TRISE;
	I2C1->TRISE |= 17;
	
	// SET SPEED AS STANDARD MODE
	I2C1->CCR &= ~I2C_CCR_FS;
	
	// SET CLOCK SCALAR
	I2C1->CCR &= ~I2C_CCR_CCR;
	I2C1->CCR |= 107;
	
	// ENABLE I2C1
	I2C1->CR1 |= I2C_CR1_PE;
	
	// ENABLE ACK AFTER BYTE RECEPTION
	I2C1->CR1 |= I2C_CR1_ACK;
	
}

void i2c2_configuration(void) {
	
	// RESET I2C2
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
	
	// ENABLE I2C2
	I2C2->CR1 |= I2C_CR1_PE;
	
	// DO A SOFTWARE RESET
	I2C2->CR1 |= I2C_CR1_SWRST;
	I2C2->CR1 &= ~I2C_CR1_SWRST;
	
	
	/* CONFIGURE I2C2 CONTROL REGISTERS */
	
	// SET SMBBus MODE TO I2C
	I2C2->CR1 &= ~I2C_CR1_SMBUS;
	
	// CLEAR PERIPHERAL CLOCK FREQUENCY AND SET AS 2 MHz
	I2C2->CR2 &= ~I2C_CR2_FREQ;
	I2C2->CR2 |= 16;
	
	// EVENT INTERRUPT ENABLE
	I2C2->CR2 |= I2C_CR2_ITEVTEN;
	
	// DISABLE I2C TO CONFIGURE TRISE
	I2C2->CR1 &= ~I2C_CR1_PE;
	
	/* CLEAR MAX RISE TIME, SET FOR MAX
		RISE TIME OF STANDARD MODE */
	I2C2->TRISE &= ~I2C_TRISE_TRISE;
	I2C2->TRISE |= 17;
	
	// SET SPEED AS STANDARD MODE
	I2C2->CCR &= ~I2C_CCR_FS;
	
	// SET CLOCK SCALAR
	I2C2->CCR &= ~I2C_CCR_CCR;
	I2C2->CCR |= 107;
	
	// ENABLE I2C2
	I2C2->CR1 |= I2C_CR1_PE;
	
	// ENABLE ACK AFTER BYTE RECEPTION
	I2C2->CR1 |= I2C_CR1_ACK;
	
}

unsigned char reverse_char(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void delay_uS(int delay) {
	
	// DISABLE TIMER
	TIM4->CR1 &= ~TIM_CR1_CEN;
	
	// CLEAR TIMER 4 COUNTER
	TIM4->CNT = 0;
	
	// ENABLE TIMER
	TIM4->CR1 |= TIM_CR1_CEN;
	
	// WAIT DELAY MINUS OFFSET
	while(TIM4->CNT < (delay - 0));
	
}

void EXTI0_IRQHandler(void) {

	// stuff to do in interrupt
	
	// CLEAR INTERRUPT
	EXTI->PR |= EXTI_PR_PR0;
}
	
	/* SYSTEM CLOCK CONFIGURE */
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
	
	// ENABLE I2C1 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	// ENABLE I2C2 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// ENABLE GPIOB CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}
	
	/* I/O CONFIGURATION */
void io_Configure(void) {
	// ENABLE GPIO A CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// ENABLE GPIOB CLOCK
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	/* CONFIGURE I2C1 PINS */
	
	// SET GPIOB PINS 8 AND 9 AS ALTERNATE FUNCTION
	GPIOB->MODER |= ((2 << 8*2) | (2 << 9*2));
	
	// SET AS AF 4 FOR I2C2
	GPIOB->AFR[1] |= ((4 << (8-8)*4) | (4 << (9-8)*4));
	
	// SET AS OPEN DRAIN
	GPIOB->OTYPER |= ((1 << 8) | (1 << 9));
	
	// SET AS PULL UP
	GPIOB->PUPDR &= ~((3 << 8*2) | (3 << 9*2));
	GPIOB->PUPDR |= ((1 << 8*2) | (1 << 9*2));
	
	// SET OUTPUT SPEED AS HIGH SPEED
	GPIOB->OSPEEDR |= ((3 << 8*2) | (3 << 9*2));
	
	
	/* CONFIGURE I2C2 PINS */
	
	// SET GPIOB PINS 10 AND 11 AS ALTERNATE FUNCTION
	GPIOB->MODER |= ((2 << 10*2) | (2 << 11*2));
	
	// SET AS AF 4 FOR I2C2
	GPIOB->AFR[1] |= ((4 << (10-8)*4) | (4 << (11-8)*4));
	
	// SET AS OPEN DRAIN
	GPIOB->OTYPER |= ((1 << 10) | (1 << 11));
	
	// SET AS PULL UP
	GPIOB->PUPDR &= ~((3 << 10*2) | (3 << 11*2));
	GPIOB->PUPDR |= ((1 << 10*2) | (1 << 11*2));
	
	// SET OUTPUT SPEED AS HIGH SPEED
	GPIOB->OSPEEDR |= ((3 << 10*2) | (3 << 11*2));
}

	/* PUSH BUTTON INTERRUPT CONFIGURE */
void pb_Configure(void) {
	// UNMASK INTERRUPT FOR EXTI 0, PUSH BUTTON
	EXTI->IMR |= EXTI_IMR_MR0;
	// SET INTERRUPT FOR RISING EDGE
	EXTI->RTSR |= EXTI_RTSR_TR0;
	// ENABLE PUSH BUTTON INTERRUPT
	NVIC->ISER[0] |= NVIC_ISER_SETENA_6;
	
	// SET INTERRUPT PRIORITY
	NVIC->IP[6] |= (NVIC_IPR0_PRI_0 & 1);

}
	
	/* TIMER 4 CONFIGURATION */
void timer4_Configure(void){
	
	// ENABLE TIMER 4 CLOCK
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// TIMER 4 PRESCALER BASED OFF 16MHZ HSI
	TIM4->PSC |= 0;
	
	// AUTO RELOAD REGISTER
	TIM4->ARR &= ~TIM_ARR_ARR;
	TIM4->ARR |= -1;
	
	// ENABLE AUTO RELAOD
	TIM4->CR1 |= TIM_CR1_ARPE;
	
	//ENABLE TIMER
	TIM4->CR1 |= TIM_CR1_CEN;
	
}	
