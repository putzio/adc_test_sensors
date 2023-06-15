/*
 * servo_driver.h
 *
 *  Created on: May 29, 2023
 *      Author: krzysiu
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include <stdint.h>
#include "tim.h"

#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SERVO_MIN_ANGLE 60
#define SERVO_MAX_ANGLE 180
#define SERVO_STEP_TIME 30

#define SERVO_0_TIMER htim1
#define SERVO_0_CHANNEL TIM_CHANNEL_1
#define SERVO_1_TIMER htim1
#define SERVO_1_CHANNEL TIM_CHANNEL_4
#define SERVO_2_TIMER htim1
#define SERVO_2_CHANNEL TIM_CHANNEL_3
#define SERVO_3_TIMER htim1
#define SERVO_3_CHANNEL TIM_CHANNEL_2



typedef struct Servo {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	uint8_t targetPosition;
	uint8_t currentPosition;
	uint8_t stepTime;
//	uint16_t thresholdForce;
} Servo;

void GRP_InitialiseServo(Servo *servo, TIM_HandleTypeDef *timer, uint32_t channel, uint8_t currentPosition);
void GRP_SetServo(Servo *servo, uint8_t angle);
void GRP_EnableServo(Servo *servo);
void GRP_DisableServo(Servo *servo);
void GRP_GoToPositionServo(Servo *servo);

#endif /* INC_SERVO_DRIVER_H_ */
