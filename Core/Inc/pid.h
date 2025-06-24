/*
 * pid.h
 *
 *  Created on: 31/03/2023
 *      Author: JV4K
 */

#ifndef INC_REG_H_
#define INC_REG_H_

#endif /* INC_REG_H_ */

#include "main.h"

typedef struct {
	float error;
	float kp;
	float ki;
	float kd;

	// If set, represents a gain of integral component anti-windup algorithm
	float kt;

	// If set, controller neglects error in range of [-toleranceBand; +toleranceBand]
	float toleranceBand;

	// PID components
	float P;
	float I;
	float D;

	// Sampling period in seconds
	float dt;

	// Output limits
	float upperLimit;
	float lowerLimit;

	float rawOutput; // No saturation
	float output; // Final saturated output signal

	float previousError;
} pid_t;

// Must be called with specified update period
void pid_calculate(pid_t *pid, float setpoint, float feedback);

// Resets all the components and previous error
void pid_reset(pid_t *pid);

// Getter for output
float pid_getOutput(pid_t *pid);

// Initialization with gains and function call frequency in herz
void pid_init(pid_t *pid, float Kp, float Ki, float Kd, float Frequency);

// Setter for gains
void pid_setGains(pid_t *pid, float Kp, float Ki, float Kd);

// Integral component anti-windup gain
void pid_setAntiWindup(pid_t *pid, float Kt);

// Setters for output limits
void pid_setLimits(pid_t *pid, float UpperLimit, float LowerLimit);
void pid_setUpperLimit(pid_t *pid, float UpperLimit);
void pid_setLowerLimit(pid_t *pid, float LowerLimit);

// Tolerance band setter
void pid_setToleranceBand(pid_t *pid, float ToleranceBand);
