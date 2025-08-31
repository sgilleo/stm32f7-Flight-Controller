/*
 * Servo.h
 *
 *  Created on: Aug 13, 2025
 *      Author: sergi
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f7xx_hal.h"

void Servo_Begin(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3);

void Servo_Move(TIM_HandleTypeDef *timerHandle, int channel, float pulse_width_us);

void Servo_Update(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, float *outputs);

#endif /* INC_SERVO_H_ */
