#ifndef MOTOR_H
#define MOTOR_H

#define ARM_MATH_CM4
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>

#define iGain 500.0f
#define pGain .05f
#define dGain .000000000005f
#define epsilon .01f
#define delta_t .000020f // update period
#define MAX 12.0f
#define MIN -12.0f

#define ARR 800
#define ntaps 101
#define blocksize 50
#define numStages 3

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
	
	// CURRENT DATA
	float Current;
	
	// PID DATA
	float integral;
	float prev_error;
	
	// TIMER DATA
	char wheel;
	float duty_cycle;
	
	// Low Pass FIR Filter
	arm_fir_instance_f32 filter;
	// STATE BUFFER
	float32_t pstate[ntaps + blocksize - 1];
	
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
void Motor_2_ISR(struct Motor *M);	
void Motor_3_ISR(struct Motor *M);
void Motor_4_ISR(struct Motor *M);
	
void Motor_ISR(struct Motor *M);
	
void Motor_1_Change_DCYC(struct Motor *M, float DCYC);
void Motor_2_Change_DCYC(struct Motor *M, float DCYC);
void Motor_3_Change_DCYC(struct Motor *M, float DCYC);	
void Motor_4_Change_DCYC(struct Motor *M, float DCYC);

#endif // MOTOR_H