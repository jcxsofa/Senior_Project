#include "Motor.h"

#define SHOOT 1000
float32_t fcoef[ntaps] = {0.004030, 0.004284, 0.004540, 0.004801, 0.005065, 0.005331, 0.005600, 0.005871, 0.006144, 0.006418, 0.006693, 0.006969, 0.007245, 0.007521, 0.007796, 0.008070, 0.008342, 0.008612, 0.008880, 0.009146, 0.009407, 0.009666, 0.009920, 0.010169, 0.010414, 0.010654, 0.010887, 0.011115, 0.011336, 0.011550, 0.011756, 0.011956, 0.012147, 0.012329, 0.012504, 0.012669, 0.012825, 0.012971, 0.013108, 0.013235, 0.013351, 0.013457, 0.013553, 0.013637, 0.013711, 0.013773, 0.013824, 0.013864, 0.013893, 0.013910, 0.013916, 0.013910, 0.013893, 0.013864, 0.013824, 0.013773, 0.013711, 0.013637, 0.013553, 0.013457, 0.013351, 0.013235, 0.013108, 0.012971, 0.012825, 0.012669, 0.012504, 0.012329, 0.012147, 0.011956, 0.011756, 0.011550, 0.011336, 0.011115, 0.010887, 0.010654, 0.010414, 0.010169, 0.009920, 0.009666, 0.009407, 0.009146, 0.008880, 0.008612, 0.008342, 0.008070, 0.007796, 0.007521, 0.007245, 0.006969, 0.006693, 0.006418, 0.006144, 0.005871, 0.005600, 0.005331, 0.005065, 0.004801, 0.004540, 0.004284, 0.004030};
	//float32_t input[blocksize];
//float32_t output[blocksize];
//int count = 0;

float32_t standard_coeffs [5 * numStages] = {
	0.0009,    0.0013,    0.0009, 1.6144,    -0.6666,
    1.0000,   -0.0482,    1.0000, 1.7338,    -0.8531
};
//float32_t pState[2*numStages];

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
		
		// SET PID INITIAL VALUES
		M->integral = 0;
		M->prev_error = 0;
		
		// INITIALIZED IIR FILTER FOR CURRENT SENSING
		//arm_biquad_cascade_df2T_init_f32(&M->filter, numStages, standard_coeffs, M->pState);
		// INITIALIZE FIR FILTER
		arm_fir_init_f32(&M->filter, ntaps, (float32_t *)&fcoef[0], &M->pstate[0], blocksize);		
	
	}
	
void Motor_Calc_Speed(
	struct Motor *M,
	float filter_output){
		
		float current, speed, other;
		float fudge = 2.0f;

		// CONVERT FILTER OUTPUT TO CURRENT
		filter_output /= 4095.0f;
		filter_output *= 3.3f;
		other = filter_output;
		filter_output /= 50.0f;
		filter_output /= 0.01f;
		current = filter_output * fudge;
		
		
		// DETERMINE SPEED
		//if (current  < 0.3f )
			//speed = 0;
		//else 
			speed = ((M->duty_cycle * 12.0f)-(current * (M->Resistance))) * 12.995f;
		
		// STORE INTO STRUCT
		M->BEMF_Speed = speed;
		
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
	
	if((dcyc < 0 ) && (M->Desired_Speed > 0)) dcyc = 0;
	if((dcyc > 0 ) && (M->Desired_Speed < 0)) dcyc = 0;
	
	Motor_1_Change_DCYC(M, dcyc);
}		

void Motor_2_ISR(struct Motor *M) {
	
	int CCR, OR, AND, XOR;
	float input[1];
	float output[1];
	float d_cyc;
	float32_t sum = 0;	
	
		// SELECT ADC_1 CHANNEL 5 FOR MOTOR 1
		ADC1->SQR3 |= 5;	

		// BEGIN CONVERSIONS
		ADC1->CR2 |= ADC_CR2_SWSTART;
		
		// WAIT FOR END OF CONVERSION
		while ((ADC1->SR & ADC_SR_EOC) == 0)

		// RESET INTERRUPT ENABLE
		ADC1->CR1 |= ADC_CR1_EOCIE;
		
		// CLEAR EOC AGAIN?
		ADC1->SR &= ~ADC_SR_EOC;
		
		// GET INPUT FROM ADC
		input[0] = ADC1->DR;
	
	// DO IIR FILTERING
	//arm_biquad_cascade_df2T_f32(&(M->filter), input, output, blocksize);
	
	// DO NO FILTERING
	output[0] = input[0];
	
	// DETERMINE NEW SPEED
	Motor_Calc_Speed(M, output[0]);
	
	// RUN PID LOOP
	Motor_Update_PID(M);
	
	// UPDATE TIMERS
	
	d_cyc = M->duty_cycle;
	
	if ( d_cyc >= 0 ) {
		CCR = TIM5->CCR3 &= TIM_CCR3_CCR3;
		AND = CCR & (int)(d_cyc * 4800);
		OR = CCR | (int)(d_cyc * 4800);
		XOR = AND ^ OR;
		TIM5->CCR3 ^= XOR;
	}
	
	else {
		CCR = TIM5->CCR4 &= TIM_CCR4_CCR4;
		AND = CCR & (int)(d_cyc * -4800);
		OR = CCR | (int)(d_cyc * -4800);
		XOR = AND ^ OR;
		TIM5->CCR4 ^= XOR;
	}
}

void Motor_1_ISR(struct Motor *M) {

	float input[blocksize];
	float output[blocksize];
	float average;
	float d_cyc;
	int i;
	float32_t sum = 0;	
	
		// SELECT ADC_1 CHANNEL 4 FOR MOTOR 1
		ADC1->SQR3 |= 4;	

		for(i=0; i<blocksize; i++){
	
			// BEGIN CONVERSIONS
			ADC1->CR2 |= ADC_CR2_SWSTART;
			
			// WAIT FOR END OF CONVERSION
			while ((ADC1->SR & ADC_SR_EOC) == 0)

			// RESET INTERRUPT ENABLE
			ADC1->CR1 |= ADC_CR1_EOCIE;
			
			// CLEAR EOC AGAIN?
			ADC1->SR &= ~ADC_SR_EOC;
			
			// GET INPUT FROM ADC
			input[i] = ADC1->DR;
		}
		
		
	// DO IIR FILTERING
	//arm_biquad_cascade_df2T_f32(&(M->filter), input, output, blocksize);
	// DO FIR FILTERING
	arm_fir_f32(&M->filter, input, output, blocksize);
	// DO NO FILTERING
	//output[0] = input[0];
	
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

void Motor_1_Change_DCYC(struct Motor *M, float DCYC) {
	
	float old_dcyc = M->duty_cycle;
	int CCR, OR, AND, XOR;
	int i = 0;
	
	if (DCYC > 0.0f) {
		
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
		
		CCR = TIM5->CCR1 &= TIM_CCR1_CCR1;
		AND = CCR & (int)(DCYC * ARR);
		OR = CCR | (int)(DCYC * ARR);
		XOR = AND ^ OR;
		TIM5->CCR1 ^= XOR;
		
		// UPDATE DUTYCYCLE
		M->duty_cycle = DCYC;
	}
	
	else if (DCYC < 0.0f) {
		
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
		
		CCR = TIM5->CCR2 &= TIM_CCR2_CCR2;
		AND = CCR & (int)(DCYC * -ARR);
		OR = CCR | (int)(DCYC * -ARR);
		XOR = AND ^ OR;
		TIM5->CCR2 ^= XOR;
		
		// UPDATE DUTY CYCLE
		M->duty_cycle = DCYC;	
		
	}
	
	else {
		
		// CLEAR ALL CCR VALUES
		TIM5->CCR1 = 0;
		TIM5->CCR2 = 0;
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR |= (1 << 7);
		GPIOE->ODR |= (1 << 8);
		
		// UPDATE DUTY CYCLE
		M->duty_cycle = DCYC;
	}
}