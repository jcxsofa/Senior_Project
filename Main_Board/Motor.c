#include "Motor.h"

#define SHOOT 1000
float32_t fcoef[ntaps] = {0.001129, 0.001150, 0.001170, 0.001190, 0.001211, 0.001232, 0.001252, 0.001273, 0.001294, 0.001315, 0.001336, 0.001357, 0.001378, 0.001399, 0.001421, 0.001442, 0.001463, 0.001485, 0.001506, 0.001528, 0.001550, 0.001571, 0.001593, 0.001615, 0.001637, 0.001659, 0.001681, 0.001703, 0.001725, 0.001747, 0.001769, 0.001791, 0.001813, 0.001835, 0.001857, 0.001879, 0.001901, 0.001923, 0.001946, 0.001968, 0.001990, 0.002012, 0.002034, 0.002056, 0.002078, 0.002101, 0.002123, 0.002145, 0.002167, 0.002189, 0.002211, 0.002233, 0.002255, 0.002277, 0.002299, 0.002321, 0.002342, 0.002364, 0.002386, 0.002408, 0.002429, 0.002451, 0.002472, 0.002494, 0.002515, 0.002536, 0.002558, 0.002579, 0.002600, 0.002621, 0.002642, 0.002663, 0.002683, 0.002704, 0.002724, 0.002745, 0.002765, 0.002786, 0.002806, 0.002826, 0.002846, 0.002866, 0.002885, 0.002905, 0.002924, 0.002944, 0.002963, 0.002982, 0.003001, 0.003020, 0.003038, 0.003057, 0.003075, 0.003094, 0.003112, 0.003130, 0.003148, 0.003165, 0.003183, 0.003200, 0.003217, 0.003234, 0.003251, 0.003268, 0.003284, 0.003301, 0.003317, 0.003333, 0.003349, 0.003364, 0.003380, 0.003395, 0.003410, 0.003425, 0.003440, 0.003454, 0.003469, 0.003483, 0.003497, 0.003510, 0.003524, 0.003537, 0.003550, 0.003563, 0.003576, 0.003588, 0.003600, 0.003612, 0.003624, 0.003636, 0.003647, 0.003658, 0.003669, 0.003680, 0.003690, 0.003700, 0.003710, 0.003720, 0.003730, 0.003739, 0.003748, 0.003757, 0.003765, 0.003774, 0.003782, 0.003789, 0.003797, 0.003804, 0.003811, 0.003818, 0.003825, 0.003831, 0.003837, 0.003843, 0.003849, 0.003854, 0.003859, 0.003864, 0.003868, 0.003873, 0.003877, 0.003881, 0.003884, 0.003887, 0.003890, 0.003893, 0.003896, 0.003898, 0.003900, 0.003902, 0.003903, 0.003904, 0.003905, 0.003906, 0.003906, 0.003906, 0.003906, 0.003906, 0.003905, 0.003904, 0.003903, 0.003902, 0.003900, 0.003898, 0.003896, 0.003893, 0.003890, 0.003887, 0.003884, 0.003881, 0.003877, 0.003873, 0.003868, 0.003864, 0.003859, 0.003854, 0.003849, 0.003843, 0.003837, 0.003831, 0.003825, 0.003818, 0.003811, 0.003804, 0.003797, 0.003789, 0.003782, 0.003774, 0.003765, 0.003757, 0.003748, 0.003739, 0.003730, 0.003720, 0.003710, 0.003700, 0.003690, 0.003680, 0.003669, 0.003658, 0.003647, 0.003636, 0.003624, 0.003612, 0.003600, 0.003588, 0.003576, 0.003563, 0.003550, 0.003537, 0.003524, 0.003510, 0.003497, 0.003483, 0.003469, 0.003454, 0.003440, 0.003425, 0.003410, 0.003395, 0.003380, 0.003364, 0.003349, 0.003333, 0.003317, 0.003301, 0.003284, 0.003268, 0.003251, 0.003234, 0.003217, 0.003200, 0.003183, 0.003165, 0.003148, 0.003130, 0.003112, 0.003094, 0.003075, 0.003057, 0.003038, 0.003020, 0.003001, 0.002982, 0.002963, 0.002944, 0.002924, 0.002905, 0.002885, 0.002866, 0.002846, 0.002826, 0.002806, 0.002786, 0.002765, 0.002745, 0.002724, 0.002704, 0.002683, 0.002663, 0.002642, 0.002621, 0.002600, 0.002579, 0.002558, 0.002536, 0.002515, 0.002494, 0.002472, 0.002451, 0.002429, 0.002408, 0.002386, 0.002364, 0.002342, 0.002321, 0.002299, 0.002277, 0.002255, 0.002233, 0.002211, 0.002189, 0.002167, 0.002145, 0.002123, 0.002101, 0.002078, 0.002056, 0.002034, 0.002012, 0.001990, 0.001968, 0.001946, 0.001923, 0.001901, 0.001879, 0.001857, 0.001835, 0.001813, 0.001791, 0.001769, 0.001747, 0.001725, 0.001703, 0.001681, 0.001659, 0.001637, 0.001615, 0.001593, 0.001571, 0.001550, 0.001528, 0.001506, 0.001485, 0.001463, 0.001442, 0.001421, 0.001399, 0.001378, 0.001357, 0.001336, 0.001315, 0.001294, 0.001273, 0.001252, 0.001232, 0.001211, 0.001190, 0.001170, 0.001150, 0.001129};
	//float32_t input[blocksize];
//float32_t output[blocksize];
//int count = 0;


#define numStages 2

//float32_t standard_coeffs [5 * numStages] = {
//	0.0001,	 -0.0002,		0.0001,	 -1.9599,		0.9605,
//		1.0000,	 -1.9628,		1.0000,	 -1.9826,		0.9839};
//q31_t pCoeffs [5 * 2];
//q63_t pState [4 * 2];
//uint8_t postShift = 0;
//arm_biquad_cas_df1_32x64_ins_q31 filter;
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
		AND = CCR & (int)(DCYC * 4800);
		OR = CCR | (int)(DCYC * 4800);
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
		AND = CCR & (int)(DCYC * -4800);
		OR = CCR | (int)(DCYC * -4800);
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