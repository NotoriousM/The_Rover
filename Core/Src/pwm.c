/*
 * pwm.c
 *
 *  Created on: 28/06//2023
 *      Author: JV4K
 */
#include <pwm.h>
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define ABS(N) ((N<0)?(-N):(N))

void pwm_initDriver(pwmControl_t *driver, TIM_HandleTypeDef *htim, uint8_t pwmChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin) {
	driver->dir1_Port = dir1_Port;
	driver->dir1_Pin = dir1_Pin;

	driver->dir2_Port = dir2_Port;
	driver->dir2_Pin = dir2_Pin;

	driver->htim = htim;
	driver->timerChannel = pwmChannel;
}

void pwm_dutyLimits(pwmControl_t *driver, uint16_t minDuty, uint16_t maxDuty) {
	driver->minDuty = minDuty;
	driver->maxDuty = maxDuty;
}

void pwm_setSpeed(pwmControl_t *driver, int32_t duty) {
	if (!duty) {
		driver->_duty = 0;
		HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 0);
		HAL_GPIO_WritePin(driver->dir2_Port, driver->dir2_Pin, 0);
	} else {
		driver->_duty = constrain(ABS(duty), driver->minDuty, driver->maxDuty);
		if (duty > 0) {
			HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
			HAL_GPIO_WritePin(driver->dir2_Port, driver->dir2_Pin, 0);
		} else {
			HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 0);
			HAL_GPIO_WritePin(driver->dir2_Port, driver->dir2_Pin, 1);
		}
	}

	switch (driver->timerChannel) {
	case 1:
		driver->htim->Instance->CCR1 = driver->_duty;
		break;
	case 2:
		driver->htim->Instance->CCR2 = driver->_duty;
		break;
	case 3:
		driver->htim->Instance->CCR3 = driver->_duty;
		break;
	case 4:
		driver->htim->Instance->CCR4 = driver->_duty;
		break;
	default:
		break;
	}
}

void pwm_break(pwmControl_t *driver) {
	pwm_setSpeed(driver, 0);
	HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
	HAL_GPIO_WritePin(driver->dir1_Port, driver->dir1_Pin, 1);
}
