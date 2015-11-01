#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f407xx.h"
#include "arm_math.h"

struct Motor {
	
	// MOTOR PROPERTIES
	float No_Load_Current;
	float No_Load_Rpm;
	float Stall_Current;
	float Resistance;
	
	// SPEED DATA
	float BEMF_Speed;
	float Encoder_Speed;
	float Desired_Speed;
	
	// PID DATA
	float iGain;
	float pGain;
	float dGain;
	float integral;
	float prev_error;
	
	
	
	
	// Low Pass IIR Filter
	arm_biquad_cascade_df2T_instance_f32 filter;
	// STATE BUFFER
	float32_t pState[4];
};

void Motor_init(
	struct Motor *M,
	float No_Load_Current,
	float No_Load_Rpm,
	float Stall_Current,
	float Resistance);
	
void Motor_PID_init(
	struct Motor *M,
	float iGain,
	float pGain,
	float dGain);
	
void Motor_Update_PID(struct Motor *M);

#endif // MOTOR_H