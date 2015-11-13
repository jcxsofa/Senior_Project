#include "Motor.h"

#define SHOOT 1000
float32_t fcoef[ntaps] = {0.000026, 0.000028, 0.000031, 0.000034, 0.000036, 0.000039, 0.000042, 0.000044, 0.000047, 0.000049, 0.000052, 0.000054, 0.000056, 0.000057, 0.000059, 0.000060, 0.000060, 0.000061, 0.000061, 0.000060, 0.000059, 0.000057, 0.000055, 0.000052, 0.000049, 0.000045, 0.000040, 0.000034, 0.000028, 0.000020, 0.000012, 0.000003, -0.000007, -0.000019, -0.000031, -0.000044, -0.000058, -0.000074, -0.000090, -0.000108, -0.000127, -0.000147, -0.000168, -0.000190, -0.000213, -0.000237, -0.000263, -0.000290, -0.000317, -0.000346, -0.000375, -0.000406, -0.000437, -0.000469, -0.000502, -0.000535, -0.000569, -0.000604, -0.000639, -0.000674, -0.000709, -0.000744, -0.000780, -0.000815, -0.000850, -0.000884, -0.000918, -0.000951, -0.000983, -0.001014, -0.001043, -0.001071, -0.001098, -0.001123, -0.001146, -0.001166, -0.001185, -0.001201, -0.001214, -0.001224, -0.001231, -0.001234, -0.001234, -0.001230, -0.001223, -0.001211, -0.001195, -0.001174, -0.001149, -0.001119, -0.001083, -0.001043, -0.000996, -0.000945, -0.000887, -0.000824, -0.000755, -0.000680, -0.000598, -0.000510, -0.000415, -0.000314, -0.000206, -0.000092, 0.000029, 0.000157, 0.000292, 0.000433, 0.000582, 0.000737, 0.000899, 0.001067, 0.001242, 0.001424, 0.001612, 0.001807, 0.002007, 0.002214, 0.002427, 0.002645, 0.002869, 0.003098, 0.003333, 0.003572, 0.003816, 0.004064, 0.004316, 0.004572, 0.004831, 0.005094, 0.005359, 0.005627, 0.005896, 0.006168, 0.006441, 0.006715, 0.006989, 0.007263, 0.007538, 0.007811, 0.008084, 0.008355, 0.008624, 0.008891, 0.009155, 0.009416, 0.009673, 0.009927, 0.010175, 0.010419, 0.010658, 0.010891, 0.011118, 0.011338, 0.011552, 0.011758, 0.011957, 0.012148, 0.012330, 0.012504, 0.012669, 0.012825, 0.012972, 0.013108, 0.013235, 0.013351, 0.013457, 0.013553, 0.013637, 0.013711, 0.013773, 0.013824, 0.013864, 0.013893, 0.013910, 0.013916, 0.013910, 0.013893, 0.013864, 0.013824, 0.013773, 0.013711, 0.013637, 0.013553, 0.013457, 0.013351, 0.013235, 0.013108, 0.012972, 0.012825, 0.012669, 0.012504, 0.012330, 0.012148, 0.011957, 0.011758, 0.011552, 0.011338, 0.011118, 0.010891, 0.010658, 0.010419, 0.010175, 0.009927, 0.009673, 0.009416, 0.009155, 0.008891, 0.008624, 0.008355, 0.008084, 0.007811, 0.007538, 0.007263, 0.006989, 0.006715, 0.006441, 0.006168, 0.005896, 0.005627, 0.005359, 0.005094, 0.004831, 0.004572, 0.004316, 0.004064, 0.003816, 0.003572, 0.003333, 0.003098, 0.002869, 0.002645, 0.002427, 0.002214, 0.002007, 0.001807, 0.001612, 0.001424, 0.001242, 0.001067, 0.000899, 0.000737, 0.000582, 0.000433, 0.000292, 0.000157, 0.000029, -0.000092, -0.000206, -0.000314, -0.000415, -0.000510, -0.000598, -0.000680, -0.000755, -0.000824, -0.000887, -0.000945, -0.000996, -0.001043, -0.001083, -0.001119, -0.001149, -0.001174, -0.001195, -0.001211, -0.001223, -0.001230, -0.001234, -0.001234, -0.001231, -0.001224, -0.001214, -0.001201, -0.001185, -0.001166, -0.001146, -0.001123, -0.001098, -0.001071, -0.001043, -0.001014, -0.000983, -0.000951, -0.000918, -0.000884, -0.000850, -0.000815, -0.000780, -0.000744, -0.000709, -0.000674, -0.000639, -0.000604, -0.000569, -0.000535, -0.000502, -0.000469, -0.000437, -0.000406, -0.000375, -0.000346, -0.000317, -0.000290, -0.000263, -0.000237, -0.000213, -0.000190, -0.000168, -0.000147, -0.000127, -0.000108, -0.000090, -0.000074, -0.000058, -0.000044, -0.000031, -0.000019, -0.000007, 0.000003, 0.000012, 0.000020, 0.000028, 0.000034, 0.000040, 0.000045, 0.000049, 0.000052, 0.000055, 0.000057, 0.000059, 0.000060, 0.000061, 0.000061, 0.000060, 0.000060, 0.000059, 0.000057, 0.000056, 0.000054, 0.000052, 0.000049, 0.000047, 0.000044, 0.000042, 0.000039, 0.000036, 0.000034, 0.000031, 0.000028, 0.000026};

void Motor_init(
	struct Motor *M,
	float No_Load_Current,
	float No_Load_Rpm,
	float Stall_Current,
	float Resistance,
	char wheel) {
		
		// COPY INPUT ARGUMENTS TO STRUCT DATA
		M->No_Load_Current = No_Load_Current;
		M->No_Load_Rpm = No_Load_Rpm;
		M->Stall_Current = Stall_Current;
		M->Resistance = Resistance;
		M->wheel = wheel;
		
		// SET INITIAL DUTYCYCLE
		M->duty_cycle = 0;
		
		// INTIALIZE SPEED DATA VALUES
		M->BEMF_Speed = 0;
		M->Encoder_Speed = 0;
		M->Desired_Speed = 0;
		
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
		float fudge = 2.0f;

		// CONVERT FILTER OUTPUT TO CURRENT
		filter_output /= 4095.0f;
		filter_output *= 3.3f;
		filter_output /= 50.0f;
		filter_output /= 0.01f;
		current = filter_output * fudge;
		
		// CALCULATE SPEED
		speed = ((M->duty_cycle * 12.0f)-(current * (M->Resistance))) * 12.995f;
		
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
			Motor_2_Change_DCYC(M, dcyc);
			break;
		
		case 3:
			Motor_3_Change_DCYC(M, dcyc);
			break;
		
		case 4:
			Motor_4_Change_DCYC(M, dcyc);
			break;
	}
}		

void Motor_1_Change_DCYC(struct Motor *M, float DCYC) {
	
	float old_dcyc = M->duty_cycle;
	int CCR, OR, AND, XOR;
	int i = 0;
	
	if (DCYC > 0.0f) {
		
		// CHECK TO SEE IF CHANGING DIRECTION
		if (old_dcyc < 0.0f) {
			/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
			
			// CLEAR ALL CCR VALUES
			TIM5->CCR1 = 0;
			TIM5->CCR2 = 0;
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR |= (1 << 7);
			GPIOE->ODR |= (1 << 8);
			
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR |= (1 << 7);
			GPIOE->ODR |= (1 << 8);
			
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
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR |= (1 << 7);
		GPIOE->ODR |= (1 << 8);
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR |= (1 << 9);
			GPIOE->ODR |= (1 << 10);
			
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR |= (1 << 9);
			GPIOE->ODR |= (1 << 10);
			
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
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR |= (1 << 9);
		GPIOE->ODR |= (1 << 10);
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR |= (1 << 3);
			GPIOE->ODR |= (1 << 4);
			
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOE->ODR |= (1 << 3);
			GPIOE->ODR |= (1 << 4);
			
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
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR |= (1 << 3);
		GPIOE->ODR |= (1 << 4);
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOA->ODR |= (1 << 8);
			GPIOC->ODR |= (1 << 8);
			
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
			
			// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
			GPIOA->ODR |= (1 << 8);
			GPIOC->ODR |= (1 << 8);
			
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
		GPIOA->ODR |= (1 << 8);
		GPIOC->ODR |= (1 << 8);
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
	}
	average /= blocksize;
	
	// DETERMINE NEW SPEED
	Motor_Calc_Speed(M, average);
	
	// RUN PID LOOP
	Motor_Update_PID(M);
	
}
