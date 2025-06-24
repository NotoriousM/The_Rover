/*
 * servocontroller.c
 *
 *  Created on: Jun 29, 2023
 *      Author: JV4K
 */

#include <servocontroller.h>
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*
 Initializes servo with controller architecture and motor specifications.

 Arguments:
 1. Instance of servo struct type

 2. Controller architecture, possible configurations:
 Single - position controller only
 Double - position and velocity controllers
 Triple - position, velocity and current controllers

 3. Speed of motor before gearbox in rad/s

 4. Gear ratio of reducer (e.g. if reducer is 1:21.3, pass 21.3)
 */
void servo_baseInit(servocontrol_t *servo, enum loops servoLoops, float motorSpeed, float gearRatio,
		uint8_t reverse) {
	servo->controllerLoops = servoLoops;
	servo->encoder.gearRatio = gearRatio;
	if (reverse) {
		servo->reverseFlag = -1;
	}
	else{
		servo->reverseFlag = 1;
	}
	servo->maxShaftSpeed = motorSpeed / gearRatio;

	if (servoLoops != Single) {
		servo->pid_position.lowerLimit = -servo->maxShaftSpeed;
		servo->pid_position.upperLimit = servo->maxShaftSpeed;
	}
}

/*
 Initializes servo with encoder settings.

 htim - pointer to handler of timer in encoder mode.
 CPR - counts per revolution. If you are using both channels and rising/falling edges of encoder, CPR=PPR*4.
 */
void servo_encoderInit(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint16_t CPR) {
	servo->encoder.htim = htim;
	servo->encoder.countsPerRevolution = CPR;
}

/*
 Initializes driver control module.
 Arguments:
 1. Instance of servo struct
 2. Handler of timer in pwm generation mode
 3. No of channel (1/2/3/4)
 4,5. Port and pin of output A
 6,7. Port and pin of output B
 8,9. Min and max duty-cycle of PWM

 GPIOs are needed to control direction with a dc-motor driver
 */
void servo_driverInit(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint8_t timerChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin,
		uint16_t minDuty, uint16_t maxDuty) {
	servo->driver.htim = htim;
	servo->driver.timerChannel = timerChannel;
	servo->driver.dir1_Port = dir1_Port;
	servo->driver.dir1_Pin = dir1_Pin;
	servo->driver.dir2_Port = dir2_Port;
	servo->driver.dir2_Pin = dir2_Pin;

	servo->driver.minDuty = minDuty;
	servo->driver.maxDuty = maxDuty;

	switch (servo->controllerLoops) {
	case Single:
		servo->pid_position.lowerLimit = -maxDuty;
		servo->pid_position.upperLimit = maxDuty;
		break;
	case Double:
		servo->pid_velocity.lowerLimit = -maxDuty;
		servo->pid_velocity.upperLimit = maxDuty;
		break;
	case Triple:
		servo->pid_current.lowerLimit = -maxDuty;
		servo->pid_current.upperLimit = maxDuty;
		break;
	default:
		break;
	}
}

// Initialization of controller loops with PID gains and period
void servo_positionInit(servocontrol_t *servo, float kp, float ki, float kd, float dt, float kt) {
	servo->pid_position.kp = kp;
	servo->pid_position.ki = ki;
	servo->pid_position.kd = kd;
	servo->pid_position.dt = dt;
	servo->pid_position.kt = kt;
}
void servo_velocityInit(servocontrol_t *servo, float kp, float ki, float kd, float dt, float kt) {
	servo->pid_velocity.kp = kp;
	servo->pid_velocity.ki = ki;
	servo->pid_velocity.kd = kd;
	servo->pid_velocity.dt = dt;
	servo->pid_velocity.kt = kt;
	servo->encoder.dt = dt;
}
void servo_currentInit(servocontrol_t *servo, float ratedCurrent, float kp, float ki, float kd, float dt,
		float kt) {
	servo->pid_current.kp = kp;
	servo->pid_current.ki = ki;
	servo->pid_current.kd = kd;
	servo->pid_current.dt = dt;
	servo->pid_current.kt = kt;
	if (servo->controllerLoops == Triple) {
		servo->pid_velocity.lowerLimit = -ratedCurrent;
		servo->pid_velocity.upperLimit = ratedCurrent;
	}
}

// Specifies tolerated error of shaft angle
void servo_setPositionTolerance(servocontrol_t *servo, float tolerance) {
	servo->pid_position.toleranceBand = tolerance;
}

// Returns True if error is present, False if not
int servo_getState(servocontrol_t *servo) {
	if (servo->pid_position.error == 0) {
		return 0;
	}
	return 1;
}

/*
 Returns direction of shaft rotation:
 -1 : moving backward
 0  : idle
 1  : moving forward
 */
int servo_getDirection(servocontrol_t *servo) {
	if (servo->encoder.angularVelocity > 0) {
		return 1*servo->reverseFlag;
	}
	if (servo->encoder.angularVelocity < 0) {
		return -1*servo->reverseFlag;
	}
	return 0;
}

/*
 Position controller loop. Contains PID controller and algorithm which decides whether output should be passed
 to velocity loop (if system is Double/Triple-loop) or used as pwm duty cycle (Single-loop).

 Must be called with specified period for position controller.
 */
void servo_positionLoop(servocontrol_t *servo) {
	encoder_updatePosition(&servo->encoder);

	switch (servo->currentMode) {
	case Position: {
		pid_calculate(&servo->pid_position, servo->positionSetpoint, encoder_getAngle(&servo->encoder));
		if (servo->controllerLoops == Single) {
			pwm_setSpeed(&servo->driver, pid_getOutput(&servo->pid_position));
		} else {
			servo->velocitySetpoint = pid_getOutput(&servo->pid_position);
		}
		break;
	}
	default: {
		pid_reset(&servo->pid_position);
		servo->positionSetpoint = 0;
		break;
	}
	}
}

/*
 Current controller loop. Contains PID controller and algorithm which decides whether output should be passed
 to current loop (if system is Triple-loop) or used as pwm duty cycle (Double-loop).

 Must be called with specified period for velocity controller.
 If controller is single-loop, should not be used.
 */
void servo_velocityLoop(servocontrol_t *servo) {
	encoder_updateVelocity(&servo->encoder);
	pid_calculate(&servo->pid_velocity, servo->velocitySetpoint, encoder_getVelocity(&servo->encoder));
	switch (servo->controllerLoops) {
	case Single:
		pid_reset(&servo->pid_velocity);
		servo->velocitySetpoint = 0;
		break;
	case Double:
		pwm_setSpeed(&servo->driver, pid_getOutput(&servo->pid_velocity));
		break;
	case Triple:
		servo->currentSetpoint = pid_getOutput(&servo->pid_velocity);
		break;
	}
}

/*
 Current controller loop. Contains PID controller and PWM output.
 Must be called with specified period for current controller.

 External current feedback must be passed.
 If controller is single/double-loop, should not be used.
 */
void servo_currentLoop(servocontrol_t *servo, float currentFeedback) {
	if (servo->controllerLoops == Triple) {
		pid_calculate(&servo->pid_current, servo->currentSetpoint, currentFeedback);
		pwm_setSpeed(&servo->driver, pid_getOutput(&servo->pid_current));
	} else {
		pid_reset(&servo->pid_current);
		servo->currentSetpoint = 0;
	}
}

void servo_controlPosition(servocontrol_t *servo, float setpoint) {
	servo->currentMode = Position;
	servo->positionSetpoint = setpoint*servo->reverseFlag;
}

void servo_controlVelocity(servocontrol_t *servo, float setpoint) {
	if (servo->controllerLoops != Single) {
		servo->currentMode = Velocity;
		servo->velocitySetpoint = constrain(setpoint*servo->reverseFlag, -servo->maxShaftSpeed, servo->maxShaftSpeed);
	}
}
