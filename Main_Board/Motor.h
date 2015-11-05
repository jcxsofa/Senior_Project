#ifndef MOTOR_H
#define MOTOR_H

#define ARM_MATH_CM4
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>

#define iGain .005f
#define pGain .1f
#define dGain .01f
#define epsilon .01f
#define delta_t .01f // 100ms
#define MAX 12
#define MIN -12

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
	float integral;
	float prev_error;
	
	// TIMER DATA
	char wheel;
	float duty_cycle;
	
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
	float Resistance,
	char wheel);
	
void Motor_Calc_Speed(
	struct Motor *M,
	float filter_output);
	
void Motor_Update_PID(struct Motor *M);
	
void Motor_1_ISR(struct Motor *M);

#endif // MOTOR_H