/*
 * pwm.h
 *
 *  Created on: 28/06//2023
 *      Author: JV4K
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#endif /* INC_PWM_H_ */

#include <main.h>

typedef struct {
	// Timer handler
	TIM_HandleTypeDef *htim;
	uint8_t timerChannel;

	// Direction pins and ports
	uint32_t dir1_Pin;
	GPIO_TypeDef *dir1_Port;

	uint32_t dir2_Pin;
	GPIO_TypeDef *dir2_Port;

	uint16_t maxDuty;
	uint16_t minDuty;

	int32_t _duty;

} pwmControl_t;

void pwm_initDriver(pwmControl_t *driver, TIM_HandleTypeDef *htim, uint8_t pwmChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin);

void pwm_dutyLimits(pwmControl_t *driver, uint16_t minDuty, uint16_t maxDuty);

void pwm_setSpeed(pwmControl_t *driver, int32_t duty);

void pwm_break(pwmControl_t *driver);
