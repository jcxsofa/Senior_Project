#include "Motor.h"

#define SHOOT 1000
float32_t fcoef[ntaps] = {0.004030, 0.004284, 0.004540, 0.004801, 0.005065, 0.005331, 0.005600, 0.005871, 0.006144, 0.006418, 0.006693, 0.006969, 0.007245, 0.007521, 0.007796, 0.008070, 0.008342, 0.008612, 0.008880, 0.009146, 0.009407, 0.009666, 0.009920, 0.010169, 0.010414, 0.010654, 0.010887, 0.011115, 0.011336, 0.011550, 0.011756, 0.011956, 0.012147, 0.012329, 0.012504, 0.012669, 0.012825, 0.012971, 0.013108, 0.013235, 0.013351, 0.013457, 0.013553, 0.013637, 0.013711, 0.013773, 0.013824, 0.013864, 0.013893, 0.013910, 0.013916, 0.013910, 0.013893, 0.013864, 0.013824, 0.013773, 0.013711, 0.013637, 0.013553, 0.013457, 0.013351, 0.013235, 0.013108, 0.012971, 0.012825, 0.012669, 0.012504, 0.012329, 0.012147, 0.011956, 0.011756, 0.011550, 0.011336, 0.011115, 0.010887, 0.010654, 0.010414, 0.010169, 0.009920, 0.009666, 0.009407, 0.009146, 0.008880, 0.008612, 0.008342, 0.008070, 0.007796, 0.007521, 0.007245, 0.006969, 0.006693, 0.006418, 0.006144, 0.005871, 0.005600, 0.005331, 0.005065, 0.004801, 0.004540, 0.004284, 0.004030};

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

void Motor_2_Change_DCYC(struct Motor *M, float DCYC) {
	
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
		CCR = TIM5->CCR3 &= TIM_CCR3_CCR3;
		AND = CCR & (int)(DCYC * ARR);
		OR = CCR | (int)(DCYC * ARR);
		XOR = AND ^ OR;
		TIM5->CCR3 ^= XOR;
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
		GPIOE->ODR &= ~(1 << 9);
		GPIOE->ODR |= (1 << 10);
		
		// CALCULATE NEW CCR VALUE
		CCR = TIM5->CCR4 &= TIM_CCR4_CCR4;
		AND = CCR & (int)(DCYC * -ARR);
		OR = CCR | (int)(DCYC * -ARR);
		XOR = AND ^ OR;
		TIM5->CCR4 ^= XOR;		
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

void Motor_3_Change_DCYC(struct Motor *M, float DCYC) {
	
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
		GPIOE->ODR &= ~(1 << 3);
		GPIOE->ODR |= (1 << 4);
		
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
			GPIOD->ODR |= (1 << 8);
			GPIOD->ODR |= (1 << 9);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN FORWARDS
		GPIOD->ODR &= ~(1 << 8);
		GPIOD->ODR |= (1 << 9);
		
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
			GPIOD->ODR |= (1 << 8);
			GPIOD->ODR |= (1 << 9);
			
			// DEAD TIME TO PREVENT SHOOT THROUGH
			for (i=0; i<SHOOT; i++);
		}
		
		// SETUP LOWER GATES TO RUN BACKWARDS
		GPIOD->ODR &= ~(1 << 8);
		GPIOD->ODR |= (1 << 9);
		
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
		GPIOD->ODR |= (1 << 8);
		GPIOD->ODR |= (1 << 9);
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
		channel = (M->Desired_Speed > 0) ? 4 : 5;
			break;
		
		case 2:
			ADCx = ADC1;
		channel = (M->Desired_Speed > 0) ? 6 : 7;
			break;
		
		case 3:
			ADCx = ADC2;
		channel = (M->Desired_Speed > 0) ? 8 : 9;
			break;
		
		case 4:
			ADCx = ADC2;
		channel = (M->Desired_Speed > 0) ? 10 : 11;
			break;
	}
	
	// SELECT ADC_1 CHANNEl FOR MOTOR DIRECTION
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
