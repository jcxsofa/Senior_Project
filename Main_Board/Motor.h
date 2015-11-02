#ifndef MOTOR_H
#define MOTOR_H

#define ARM_MATH_CM4
#include "stm32f407xx.h"
#include "arm_math.h"
#include <stdlib.h>

#define iGain .005
#define pGain .1
#define dGain .01
#define epsilon .01
#define delta_t .01 // 100ms
#define MAX 4
#define MIN -4

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
	
	// ASSOCIATED TIMER AND CHANNEL
	TIM_TypeDef * TIMx;
	char channel;
	
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
	
void Motor_PWM_init(
	struct Motor *M,
	TIM_TypeDef * TIMx,
	// CHANNEL USED FOR FORWARD MOTION
	char channel);
	
void Motor_Update_PID(struct Motor *M);

#endif // MOTOR_H