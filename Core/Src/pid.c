/*
 * pid.c
 *
 *  Created on: 31/03/2023
 *      Author: JV4K
 */

#include <pid.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Calculates output based on passed setpoint and feedback, must be called with specified period
void pid_calculate(pid_t *pid, float setpoint, float feedback) {
	pid->error = setpoint - feedback;

	if ((pid->error > -(pid->toleranceBand)) && (pid->error < pid->toleranceBand)) {
		pid->error = 0;
		pid_reset(pid);
	}

	pid->P = pid->error * pid->kp;
	pid->I += (pid->output - pid->rawOutput) * pid->kt + pid->error * pid->dt * pid->ki;
	pid->D = ((pid->error - pid->previousError) * pid->kd) / pid->dt;

	pid->rawOutput = pid->P + pid->I + pid->D;
	pid->output = constrain(pid->rawOutput, pid->lowerLimit, pid->upperLimit);
	pid->previousError = pid->error;
}

// Resets all the components and previous error
void pid_reset(pid_t *pid) {
	pid->P = 0;
	pid->I = 0;
	pid->D = 0;
	pid->previousError = 0;
	pid->output = 0;
}

float pid_getOutput(pid_t *pid) {
	return pid->output;
}

// Initialization with gains and update period
void pid_init(pid_t *pid, float Kp, float Ki, float Kd, float Frequency) {
	pid->kp = Kp;
	pid->ki = Ki;
	pid->kd = Kd;
	pid->dt = (float) 1 / Frequency;
}

void pid_setGains(pid_t *pid, float Kp, float Ki, float Kd) {
	pid->kp = Kp;
	pid->ki = Ki;
	pid->kd = Kd;
}

void pid_setAntiWindup(pid_t *pid, float Kt) {
	pid->kt = Kt;
}

void pid_setLimits(pid_t *pid, float LowerLimit, float UpperLimit) {
	pid->lowerLimit = LowerLimit;
	pid->upperLimit = UpperLimit;
}

void pid_setUpperLimit(pid_t *pid, float UpperLimit) {
	pid->upperLimit = UpperLimit;
}

void pid_setLowerLimit(pid_t *pid, float LowerLimit) {
	pid->lowerLimit = LowerLimit;
}

void pid_setToleranceBand(pid_t *pid, float ToleranceBand) {
	pid->toleranceBand = ToleranceBand;
}
