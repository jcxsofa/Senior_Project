#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f407xx.h"
#include "arm_math.h"

struct Motor {
	float No_Load_Current;
	float No_Load_Rpm;
	float Stall_Current;
	float Resistance;
	float BEMF_Speed;
	float Encoder_Speed;
	float Desired_Speed;
	// Low Pass IIR Filter
	arm_biquad_cascade_df2T_instance_f32 *filter;
	// STATE BUFFER
	float32_t pState[4];
};

void Motor_init(
	struct Motor *M,
	float No_Load_Current,
	float No_Load_Rpm,
	float Stall_Current,
	float Resistance);

#endif // MOTOR_H