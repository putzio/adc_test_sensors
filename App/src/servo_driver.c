/*
 * servo_driver.c
 *
 *  Created on: May 29, 2023
 *      Author: krzysiu
 */
#include "../../App/inc/servo_driver.h"

static inline uint16_t MapAngleToPwmMs(uint8_t angle) {
	return angle * (SERVO_MAX - SERVO_MIN) / 180 + SERVO_MIN;
}

void SRV_Initialise(Servo* servo, TIM_HandleTypeDef* timer, uint32_t channel, uint8_t currentPosition) {
	servo->timer = timer;
	servo->channel = channel;
	servo->stepTime = SERVO_STEP_TIME;
	servo->currentPosition = currentPosition;
	HAL_TIM_PWM_Start(servo->timer, servo->channel);
}

void SRV_Set(Servo* servo, uint8_t angle) {
	__HAL_TIM_SET_COMPARE(servo->timer, servo->channel, MapAngleToPwmMs(angle));
}

void SRV_Enable(Servo* servo) {
	HAL_TIM_Base_Start_IT(servo->timer);
}

void SRV_Disable(Servo* servo) {
	__HAL_TIM_SET_COMPARE(servo->timer, servo->channel, 0);
	HAL_TIM_Base_Stop_IT(servo->timer);
}

void SRV_GoToPosition(Servo* servo) {
	if (servo->targetPosition > servo->currentPosition) {
		servo->currentPosition += 1;
	}
	else if (servo->targetPosition < servo->currentPosition) {
		servo->currentPosition -= 1;
	}
	SRV_Set(servo, servo->currentPosition);
}


