#include "Motor.h"

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
	0.0001,	 -0.0002,		0.0001,	 -1.9599,		0.9605,
		1.0000,	 -1.9628,		1.0000,	 -1.9826,		0.9839};
//q31_t pCoeffs [5 * 2];
//q63_t pState [4 * 2];
//uint8_t postShift = 0;
//arm_biquad_cas_df1_32x64_ins_q31 filter;
float32_t pState[2*numStages];

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
		arm_biquad_cascade_df2T_init_f32(&M->filter, numStages, standard_coeffs, M->pState); 
	
	}
	
void Motor_Calc_Speed(
	struct Motor *M,
	float filter_output){
		
		float current, speed;

		// CONVERT FILTER OUTPUT TO CURRENT
		filter_output /= 0xFFF;
		filter_output *= 3;
		filter_output /= 50;
		current = filter_output / 0.01;
		
		// DETERMINE SPEED
		speed = ((M->duty_cycle)/(current * (M->Resistance))) * 10.75;
		
		// STORE INTO STRUCT
		M->BEMF_Speed = speed;
		
	}
	
void Motor_Update_PID(struct Motor *M){

	float error;
	float derivative;
	float pid_result;
	
	// CALCULATE NEW ERROR
	error = M->Desired_Speed - M->BEMF_Speed;
	
	// IF ERROR IS SMALL DON'T DO INTEGRAL
	if(abs(error) > epsilon){
		M->integral += (error*delta_t);
	}
	
	// CALCULATE DERIVATIVE TERM
	derivative = (error - M->prev_error)/delta_t;
	
	// CALCULATE PID RESULT
	pid_result = pGain*error;// + iGain*M->integral + dGain*derivative;

	if(pid_result > MAX) pid_result = MAX;
	if(pid_result < MIN) pid_result = MIN;
	
	// ASSIGN NEW ERROR AS OLD ERROR
	M->prev_error = error;
	
	// UPDATE DUTY CYCLE
	M->duty_cycle =  pid_result / MAX;
	
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

	int CCR, OR, AND, XOR;
	float input[1];
	float output[1];
	float d_cyc;
	float32_t sum = 0;	
	
		// SELECT ADC_1 CHANNEL 4 FOR MOTOR 1
		ADC1->SQR3 |= 4;	

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
		CCR = TIM5->CCR1 &= TIM_CCR1_CCR1;
		AND = CCR & (int)(d_cyc * 4800);
		OR = CCR | (int)(d_cyc * 4800);
		XOR = AND ^ OR;
		TIM5->CCR1 ^= XOR;
	}
	
	else {
		CCR = TIM5->CCR2 &= TIM_CCR2_CCR2;
		AND = CCR & (int)(d_cyc * -4800);
		OR = CCR | (int)(d_cyc * -4800);
		XOR = AND ^ OR;
		TIM5->CCR2 ^= XOR;
	}
}

void Motor_1_Change_Speed(struct Motor *M, float speed) {
	
	float old_speed = M->Desired_Speed;
	int i = 0;
	
	if ((old_speed <= 0) && (speed > 0)) {
		
		/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
		
		// CLEAR ALL CCR VALUES
		TIM5->CCR1 = 0;
		TIM5->CCR2 = 0;
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR |= (1 << 7);
		GPIOE->ODR |= (1 << 8);
		
		// DEAD TIME TO PREVENT SHOOT THROUGH
		for (i=0; i<100000; i++);
		
		// SETUP LOWER GATES TO RUN FORWARDS
		GPIOE->ODR &= ~(1 << 7);
		
		// UPDATE SPEED AND RUN ISR TO UPDATE CONTROLLER
		M->Desired_Speed = speed;
		Motor_1_ISR(M);	
	}
	
	if ((old_speed >=0) && (speed < 0)) {
		
		/* PAUSE MOTOR TO PREVENT SHOOT THROUGH */
		
		// CLEAR ALL CCR VALUES
		TIM5->CCR1 = 0;
		TIM5->CCR2 = 0;
		
		// ENABLE BOTH LOW SIDE GATES TO DRAIN CURRENT
		GPIOE->ODR |= (1 << 7);
		GPIOE->ODR |= (1 << 8);
		
		// DEAD TIME TO PREVENT SHOOT THROUGH
		for (i=0; i<100000; i++);
		
		// SETUP LOWER GATES TO RUN BACKWARDS
		GPIOE->ODR &= ~(1 << 8);
		
		// UPDATE SPEED AND RUN ISR TO UPDATE CONTROLLER
		M->Desired_Speed = speed;
		Motor_1_ISR(M);	
		
	}
}