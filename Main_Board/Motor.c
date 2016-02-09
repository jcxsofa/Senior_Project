#include "Motor.h"

#define SHOOT 100000
float32_t fcoef[ntaps] = {0.000960, 0.000966, 0.000972, 0.000978, 0.000984, 0.000990, 0.000996, 0.001002, 0.001008, 0.001014, 0.001020, 0.001026, 0.001032, 0.001038, 0.001044, 0.001050, 0.001056, 0.001062, 0.001068, 0.001074, 0.001080, 0.001086, 0.001092, 0.001098, 0.001104, 0.001110, 0.001116, 0.001122, 0.001128, 0.001134, 0.001140, 0.001146, 0.001152, 0.001158, 0.001164, 0.001170, 0.001176, 0.001182, 0.001188, 0.001194, 0.001200, 0.001206, 0.001212, 0.001218, 0.001224, 0.001230, 0.001236, 0.001242, 0.001247, 0.001253, 0.001259, 0.001265, 0.001271, 0.001277, 0.001283, 0.001288, 0.001294, 0.001300, 0.001306, 0.001311, 0.001317, 0.001323, 0.001329, 0.001334, 0.001340, 0.001346, 0.001352, 0.001357, 0.001363, 0.001369, 0.001374, 0.001380, 0.001385, 0.001391, 0.001397, 0.001402, 0.001408, 0.001413, 0.001419, 0.001424, 0.001430, 0.001435, 0.001441, 0.001446, 0.001451, 0.001457, 0.001462, 0.001468, 0.001473, 0.001478, 0.001484, 0.001489, 0.001494, 0.001499, 0.001505, 0.001510, 0.001515, 0.001520, 0.001525, 0.001530, 0.001535, 0.001541, 0.001546, 0.001551, 0.001556, 0.001561, 0.001566, 0.001571, 0.001575, 0.001580, 0.001585, 0.001590, 0.001595, 0.001600, 0.001605, 0.001609, 0.001614, 0.001619, 0.001623, 0.001628, 0.001633, 0.001637, 0.001642, 0.001646, 0.001651, 0.001655, 0.001660, 0.001664, 0.001669, 0.001673, 0.001678, 0.001682, 0.001686, 0.001690, 0.001695, 0.001699, 0.001703, 0.001707, 0.001711, 0.001716, 0.001720, 0.001724, 0.001728, 0.001732, 0.001736, 0.001740, 0.001743, 0.001747, 0.001751, 0.001755, 0.001759, 0.001762, 0.001766, 0.001770, 0.001773, 0.001777, 0.001781, 0.001784, 0.001788, 0.001791, 0.001795, 0.001798, 0.001801, 0.001805, 0.001808, 0.001811, 0.001815, 0.001818, 0.001821, 0.001824, 0.001827, 0.001830, 0.001833, 0.001836, 0.001839, 0.001842, 0.001845, 0.001848, 0.001851, 0.001854, 0.001856, 0.001859, 0.001862, 0.001864, 0.001867, 0.001870, 0.001872, 0.001875, 0.001877, 0.001879, 0.001882, 0.001884, 0.001886, 0.001889, 0.001891, 0.001893, 0.001895, 0.001897, 0.001900, 0.001902, 0.001904, 0.001906, 0.001907, 0.001909, 0.001911, 0.001913, 0.001915, 0.001916, 0.001918, 0.001920, 0.001921, 0.001923, 0.001925, 0.001926, 0.001928, 0.001929, 0.001930, 0.001932, 0.001933, 0.001934, 0.001935, 0.001937, 0.001938, 0.001939, 0.001940, 0.001941, 0.001942, 0.001943, 0.001944, 0.001945, 0.001945, 0.001946, 0.001947, 0.001948, 0.001948, 0.001949, 0.001949, 0.001950, 0.001950, 0.001951, 0.001951, 0.001952, 0.001952, 0.001952, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001953, 0.001952, 0.001952, 0.001952, 0.001951, 0.001951, 0.001950, 0.001950, 0.001949, 0.001949, 0.001948, 0.001948, 0.001947, 0.001946, 0.001945, 0.001945, 0.001944, 0.001943, 0.001942, 0.001941, 0.001940, 0.001939, 0.001938, 0.001937, 0.001935, 0.001934, 0.001933, 0.001932, 0.001930, 0.001929, 0.001928, 0.001926, 0.001925, 0.001923, 0.001921, 0.001920, 0.001918, 0.001916, 0.001915, 0.001913, 0.001911, 0.001909, 0.001907, 0.001906, 0.001904, 0.001902, 0.001900, 0.001897, 0.001895, 0.001893, 0.001891, 0.001889, 0.001886, 0.001884, 0.001882, 0.001879, 0.001877, 0.001875, 0.001872, 0.001870, 0.001867, 0.001864, 0.001862, 0.001859, 0.001856, 0.001854, 0.001851, 0.001848, 0.001845, 0.001842, 0.001839, 0.001836, 0.001833, 0.001830, 0.001827, 0.001824, 0.001821, 0.001818, 0.001815, 0.001811, 0.001808, 0.001805, 0.001801, 0.001798, 0.001795, 0.001791, 0.001788, 0.001784, 0.001781, 0.001777, 0.001773, 0.001770, 0.001766, 0.001762, 0.001759, 0.001755, 0.001751, 0.001747, 0.001743, 0.001740, 0.001736, 0.001732, 0.001728, 0.001724, 0.001720, 0.001716, 0.001711, 0.001707, 0.001703, 0.001699, 0.001695, 0.001690, 0.001686, 0.001682, 0.001678, 0.001673, 0.001669, 0.001664, 0.001660, 0.001655, 0.001651, 0.001646, 0.001642, 0.001637, 0.001633, 0.001628, 0.001623, 0.001619, 0.001614, 0.001609, 0.001605, 0.001600, 0.001595, 0.001590, 0.001585, 0.001580, 0.001575, 0.001571, 0.001566, 0.001561, 0.001556, 0.001551, 0.001546, 0.001541, 0.001535, 0.001530, 0.001525, 0.001520, 0.001515, 0.001510, 0.001505, 0.001499, 0.001494, 0.001489, 0.001484, 0.001478, 0.001473, 0.001468, 0.001462, 0.001457, 0.001451, 0.001446, 0.001441, 0.001435, 0.001430, 0.001424, 0.001419, 0.001413, 0.001408, 0.001402, 0.001397, 0.001391, 0.001385, 0.001380, 0.001374, 0.001369, 0.001363, 0.001357, 0.001352, 0.001346, 0.001340, 0.001334, 0.001329, 0.001323, 0.001317, 0.001311, 0.001306, 0.001300, 0.001294, 0.001288, 0.001283, 0.001277, 0.001271, 0.001265, 0.001259, 0.001253, 0.001247, 0.001242, 0.001236, 0.001230, 0.001224, 0.001218, 0.001212, 0.001206, 0.001200, 0.001194, 0.001188, 0.001182, 0.001176, 0.001170, 0.001164, 0.001158, 0.001152, 0.001146, 0.001140, 0.001134, 0.001128, 0.001122, 0.001116, 0.001110, 0.001104, 0.001098, 0.001092, 0.001086, 0.001080, 0.001074, 0.001068, 0.001062, 0.001056, 0.001050, 0.001044, 0.001038, 0.001032, 0.001026, 0.001020, 0.001014, 0.001008, 0.001002, 0.000996, 0.000990, 0.000984, 0.000978, 0.000972, 0.000966, 0.000960};

void Motor_init(
	struct Motor *M,
	float No_Load_Current,
	float No_Load_Rpm,
	float Stall_Current,
	float Resistance,
	float kv,
	char wheel,
	float help) {
		
		// COPY INPUT ARGUMENTS TO STRUCT DATA
		M->No_Load_Current = No_Load_Current;
		M->No_Load_Rpm = No_Load_Rpm;
		M->Stall_Current = Stall_Current;
		M->Resistance = Resistance;
		M->wheel = wheel;
		M->kv = kv;
		
		// SET INITIAL DUTYCYCLE
		M->duty_cycle = 0;
		
		// INTIALIZE SPEED DATA VALUES
		M->BEMF_Speed = 0;
		M->Encoder_Speed = 0;
		M->Desired_Speed = 0;
		M->Error = 0;
		M->OldEncoder = 0;
		M->helpless = help;
		
		// INTIALIZE CURRENT VALUE
		M->Current = 0;
		
		// SET PID INITIAL VALUES
		M->integral = 0;
		M->prev_error = 0;
		
		// INITIALIZE FIR FILTER
		arm_fir_init_f32(&M->filter, ntaps, (float32_t *)&fcoef[0], &M->pstate[0], blocksize);		
	
	}
	
	
void Motor_Calc_Speed(
	struct Motor *M,
	float filter_output){
		
		float current, speed;
		float calibration = 4.92f;

		// CONVERT FILTER OUTPUT TO CURRENT
		filter_output /= 4095.0f;
		filter_output *= 3.3f;
		filter_output /= 50.0f;
		filter_output /= 0.01f;
		current = filter_output * calibration * M->helpless;
		
		// CALCULATE SPEED
		if(M->duty_cycle >= 0)
		speed = ((M->duty_cycle * 12.0f)-(current * (M->Resistance))) * M->kv;
		else speed = ((M->duty_cycle * 12.0f)+(current * (M->Resistance))) * M->kv;
		
		// STORE SPEED AND CURRENT INTO STRUCT
		M->BEMF_Speed = speed;
		M->Current = current;
		
}
	
void Motor_Update_PID(struct Motor *M){

	float error;
	float derivative;
	float pid_result;
	float dcyc;
	float integ;
	
	// CALCULATE NEW ERROR
	error = M->Desired_Speed - M->BEMF_Speed;
	
	// IF ERROR IS SMALL DON'T DO INTEGRAL
	if(abs(error) > epsilon){
		M->integral += (error*delta_t);
	}
	
	// CALCULATE DERIVATIVE TERM
	derivative = (error - M->prev_error)/delta_t;
	
	// INTEG
	integ = iGain*(M->integral);
	
	// CALCULATE PID RESULT
	pid_result = pGain*error + integ;// + dGain*derivative;

	if(pid_result > MAX) pid_result = MAX;
	if(pid_result < MIN) pid_result = MIN;
	
	// ASSIGN NEW ERROR AS OLD ERROR
	M->prev_error = error;
	
	// UPDATE DUTY CYCLE
	dcyc = pid_result / MAX;
	
	// DONT BACKDRIVE MOTOR RELATIVE TO DESIRED DIRECTION
	if((dcyc < 0 ) && (M->Desired_Speed > 0)) dcyc = 0;
	if((dcyc > 0 ) && (M->Desired_Speed < 0)) dcyc = 0;
	
	// CALL THE CORRECT FUNCTION FOR EACH WHEEL
	switch (M->wheel)
	{
		case 1:
			Motor_1_Change_DCYC(M, dcyc);
			break;
		
		case 2:
			//Motor_2_Change_DCYC(M, dcyc);
			break;
		
		case 3:
			//Motor_3_Change_DCYC(M, dcyc);
			break;
		
		case 4:
			//Motor_4_Change_DCYC(M, dcyc);
			break;
	}
}		

void Motor_1_Change_DCYC(struct Motor *M, float DCYC) {
	
	float old_dcyc = M->duty_cycle;
	int CCR, OR, AND, XOR;
	int i = 0;
	
	DCYC = DCYC * M->helpless;
	
	if (DCYC > 0.0f) {
		
		// CHECK TO SEE IF CHANGING DIRECTION
		if (old_dcyc < 0.0f) {
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM5->CCR1 = 0;
			TIM5->CCR2 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR &= ~(1 << 7);
			GPIOE->ODR &= ~(1 << 8);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN FORWARDS
		GPIOE->ODR &= ~(1 << 7);
		GPIOE->ODR |= (1 << 8);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM5->CCR1 &= TIM_CCR1_CCR1;
		AND = CCR & (int)(DCYC * ARR);
		OR = CCR | (int)(DCYC * ARR);
		XOR = AND ^ OR;
		TIM5->CCR1 ^= XOR;
	}
	
	else if (DCYC < 0.0f) {
		
		// CHECK IF CHANGING DIRECTION
		if (old_dcyc >= 0.0f) {
			
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM5->CCR1 = 0;
			TIM5->CCR2 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR &= ~(1 << 7);
			GPIOE->ODR &= ~(1 << 8);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN BACKWARDS
		GPIOE->ODR &= ~(1 << 8);
		GPIOE->ODR |= (1 << 7);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM5->CCR2 &= TIM_CCR2_CCR2;
		AND = CCR & (int)(DCYC * -ARR);
		OR = CCR | (int)(DCYC * -ARR);
		XOR = AND ^ OR;
		TIM5->CCR2 ^= XOR;		
	}
	
	else {
		
		// CLEAR ALL CCR VALUES
		TIM5->CCR1 = 0;
		TIM5->CCR2 = 0;
		
		// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR &= ~(1 << 7);
		GPIOE->ODR &= ~(1 << 8);
	}
	
	// UPDATE DUTY CYCLE
		M->duty_cycle = DCYC;
}

void Motor_3_Change_DCYC(struct Motor *M, float DCYC) {
	
	float old_dcyc = M->duty_cycle;
	int CCR, OR, AND, XOR;
	int i = 0;
	
	if (DCYC > 0.0f) {
		
		// CHECK TO SEE IF CHANGING DIRECTION
		if (old_dcyc < 0.0f) {
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM5->CCR3 = 0;
			TIM5->CCR4 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR &= ~(1 << 9);
			GPIOE->ODR &= ~(1 << 10);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN FORWARDS
		GPIOE->ODR &= ~(1 << 9);
		GPIOE->ODR |= (1 << 10);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM5->CCR4 &= TIM_CCR4_CCR4;
		AND = CCR & (int)(DCYC * ARR);
		OR = CCR | (int)(DCYC * ARR);
		XOR = AND ^ OR;
		TIM5->CCR4 ^= XOR;
	}
	
	else if (DCYC < 0.0f) {
		
		// CHECK IF CHANGING DIRECTION
		if (old_dcyc >= 0.0f) {
			
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM5->CCR3 = 0;
			TIM5->CCR4 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR &= ~(1 << 9);
			GPIOE->ODR &= ~(1 << 10);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN BACKWARDS
		GPIOE->ODR &= ~(1 << 10);
		GPIOE->ODR |= (1 << 9);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM5->CCR3 &= TIM_CCR3_CCR3;
		AND = CCR & (int)(DCYC * -ARR);
		OR = CCR | (int)(DCYC * -ARR);
		XOR = AND ^ OR;
		TIM5->CCR3 ^= XOR;		
	}
	
	else {
		
		// CLEAR ALL CCR VALUES
		TIM5->CCR3 = 0;
		TIM5->CCR4 = 0;
		
		// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR &= ~(1 << 9);
		GPIOE->ODR &= ~(1 << 10);
	}
	
	// UPDATE DUTY CYCLE
		M->duty_cycle = DCYC;
}

void Motor_2_Change_DCYC(struct Motor *M, float DCYC) {
	
	float old_dcyc = M->duty_cycle;
	int CCR, OR, AND, XOR;
	int i = 0;
	
	if (DCYC > 0.0f) {
		
		// CHECK TO SEE IF CHANGING DIRECTION
		if (old_dcyc < 0.0f) {
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM9->CCR1 = 0;
			TIM9->CCR2 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR &= ~(1 << 3);
			GPIOE->ODR &= ~(1 << 4);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN FORWARDS
		GPIOE->ODR &= ~(1 << 3);
		GPIOE->ODR |= (1 << 4);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM9->CCR1 &= TIM_CCR1_CCR1;
		AND = CCR & (int)(DCYC * ARR);
		OR = CCR | (int)(DCYC * ARR);
		XOR = AND ^ OR;
		TIM9->CCR1 ^= XOR;
	}
	
	else if (DCYC < 0.0f) {
		
		// CHECK IF CHANGING DIRECTION
		if (old_dcyc >= 0.0f) {
			
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM9->CCR1 = 0;
			TIM9->CCR2 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR &= ~(1 << 3);
			GPIOE->ODR &= ~(1 << 4);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN BACKWARDS
		GPIOE->ODR &= ~(1 << 4);
		GPIOE->ODR |= (1 << 3);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM9->CCR2 &= TIM_CCR2_CCR2;
		AND = CCR & (int)(DCYC * -ARR);
		OR = CCR | (int)(DCYC * -ARR);
		XOR = AND ^ OR;
		TIM9->CCR2 ^= XOR;		
	}
	
	else {
		
		// CLEAR ALL CCR VALUES
		TIM9->CCR1 = 0;
		TIM9->CCR2 = 0;
		
		// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR &= ~(1 << 3);
		GPIOE->ODR &= ~(1 << 4);
	}
	
	// UPDATE DUTY CYCLE
		M->duty_cycle = DCYC;
}

void Motor_4_Change_DCYC(struct Motor *M, float DCYC) {
	
	float old_dcyc = M->duty_cycle;
	int CCR, OR, AND, XOR;
	int i = 0;
	
	if (DCYC > 0.0f) {
		
		// CHECK TO SEE IF CHANGING DIRECTION
		if (old_dcyc < 0.0f) {
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM12->CCR1 = 0;
			TIM12->CCR2 = 0;
			
			// DIABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOA->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 8);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN FORWARDS
		GPIOA->ODR &= ~(1 << 8);
		GPIOC->ODR |= (1 << 8);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM12->CCR1 &= TIM_CCR1_CCR1;
		AND = CCR & (int)(DCYC * ARR);
		OR = CCR | (int)(DCYC * ARR);
		XOR = AND ^ OR;
		TIM12->CCR1 ^= XOR;
	}
	
	else if (DCYC < 0.0f) {
		
		// CHECK IF CHANGING DIRECTION
		if (old_dcyc >= 0.0f) {
			
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM12->CCR1 = 0;
			TIM12->CCR2 = 0;
			
			// DISABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOA->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 8);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN BACKWARDS
		GPIOC->ODR &= ~(1 << 8);
		GPIOA->ODR |= (1 << 8);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM12->CCR2 &= TIM_CCR2_CCR2;
		AND = CCR & (int)(DCYC * -ARR);
		OR = CCR | (int)(DCYC * -ARR);
		XOR = AND ^ OR;
		TIM12->CCR2 ^= XOR;		
	}
	
	else {
		
		// CLEAR ALL CCR VALUES
		TIM12->CCR1 = 0;
		TIM12->CCR2 = 0;
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOA->ODR &= ~(1 << 8);
		GPIOC->ODR &= ~(1 << 8);
	}
	
	// UPDATE DUTY CYCLE
		M->duty_cycle = DCYC;
}

void Motor_ISR(struct Motor *M) {
	
	ADC_TypeDef * ADCx;
	float input[blocksize];
	float output[blocksize];
	float average;
	float d_cyc;
	int i;
	int channel;

	switch (M->wheel)
	{
		case 1:
			ADCx = ADC1;
		channel = (M->Desired_Speed > 0) ? 11 : 13;
			break;
		
		case 2:
			ADCx = ADC1;
		channel = (M->Desired_Speed > 0) ? 12 : 10;
			break;
		
		case 3:
			ADCx = ADC1;
		channel = (M->Desired_Speed > 0) ? 5 : 7;
			break;
		
		case 4:
			ADCx = ADC1;
		channel = (M->Desired_Speed > 0) ? 6 : 4;
			break;
	}
	
	// SELECT ADC_1 CHANNEl FOR MOTOR DIRECTION
	ADCx->SQR3 &= ~(0x1F);
	ADCx->SQR3 |= channel;	

	for(i=0; i<blocksize; i++){

		// BEGIN CONVERSIONS
		ADCx->CR2 |= ADC_CR2_SWSTART;
		
		// WAIT FOR END OF CONVERSION
		while ((ADCx->SR & ADC_SR_EOC) == 0)

		// RESET INTERRUPT ENABLE
		ADCx->CR1 |= ADC_CR1_EOCIE;
		
		// CLEAR EOC AGAIN?
		ADCx->SR &= ~ADC_SR_EOC;
		
		// GET INPUT FROM ADC
		input[i] = ADC1->DR;
	}
		
	// DO FIR FILTERING
	arm_fir_f32(&M->filter, input, output, blocksize);
	
	// CALCULATE AVERAGE OF FILTERED VALUES
	average = 0;
	for(i=0; i<blocksize; i++) {
		average += output[i];
		if((channel == 11) || (channel == 13)) {
			// COPY DATA TO DAC OUTPUT
			DAC->DHR12R1 &= 0xFFFFF000;
			DAC->DHR12R1 |= (int)output[i];
		
		// BEGIN DAC CONVERSION
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
		}
	}
	average /= blocksize;
	
	// DETERMINE NEW SPEED
	Motor_Calc_Speed(M, average);
	
	// RUN PID LOOP
	Motor_Update_PID(M);
	
}
