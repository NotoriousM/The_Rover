/*
 * encoder.c
 *
 *  Created on: 25/06/2023
 *      Author: JV4K
 */

#include <encoder.h>

/*
 * 	Initialization of created encoder instance. Arguments:
 * 1. Pointer to encoder instance
 * 2. Pointer to handler of timer in encoder mode
 * 3. Counts per revolution of encoder (encoder PPR*4 when both channels and rising/falling edges are used)
 * 4. Gear ratio (e.g. if your motor has gear ratio of 1:21.3, pass 21.3). If no gearbox, pass 1;
 */
void encoder_init(encoder_t *encoder, TIM_HandleTypeDef *timerHandle, uint16_t CPR, float dt, float gearRatio) {
	encoder->htim = timerHandle;
	encoder->countsPerRevolution = CPR;
	encoder->dt = dt;
	if (gearRatio) {
		encoder->gearRatio = gearRatio;
	} else {
		encoder->gearRatio = 1;
	}
}

// Calculates current relative position in rad. Should be called with a specified period (dt) or less
void encoder_updatePosition(encoder_t *encoder) {
	encoder->currentTicks = (int16_t) encoder->htim->Instance->CNT;

	encoder->fullRevolutions += encoder->currentTicks / encoder->countsPerRevolution;
	encoder->currentTicks = encoder->currentTicks % encoder->countsPerRevolution;
	encoder->htim->Instance->CNT = (uint16_t) encoder->currentTicks;

	encoder->angle = (((float) encoder->fullRevolutions
			+ ((float) encoder->currentTicks / encoder->countsPerRevolution)) * 2 * M_PI)
			/ encoder->gearRatio;
}

// Calculates current angular velocity in rad/s. Must be called with a specified period (dt)
void encoder_updateVelocity(encoder_t *encoder) {
	encoder->angularVelocity = (encoder->angle - encoder->previousAngle) / encoder->dt;		// rad/s
	encoder->linearVelocity = encoder->angularVelocity * 0.1;	// m/s
	encoder->previousAngle = encoder->angle;
}

// Resets encoder state. Could be used to set a new reference point for position
void encoder_reset(encoder_t *encoder) {
	encoder->htim->Instance->CNT = 0;
	encoder->previousAngle = 0;
	encoder->angle = 0;
	encoder->fullRevolutions = 0;
	encoder->currentTicks = 0;
}

float encoder_getAngle(encoder_t *encoder){
	return encoder->angle;
}
float encoder_getVelocity(encoder_t *encoder){
	return encoder->angularVelocity;
}
