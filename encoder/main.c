#include "stm32f407xx.h"
#include "tim_1.h"

int main(void) {
	
	tim_1_gpio_init();
	tim_1_config();	
	
	while(1);
	
}
