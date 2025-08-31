/*
 * Servo.c
 *
 *  Created on: Aug 13, 2025
 *      Author: sergi
 */

#include "Servo.h"

void Servo_Begin(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3){
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_4);

}

void Servo_Move(TIM_HandleTypeDef *timerHandle, int channel, float pulse_width_us){
	if(pulse_width_us > 2000.0f) pulse_width_us = 2000.0f;
	if(pulse_width_us < 1000.0f) pulse_width_us = 1000.0f;
	uint16_t compare = pulse_width_us * 1.8;
	__HAL_TIM_SET_COMPARE(timerHandle, channel, compare);

}

void Servo_Update(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, float *outputs){
	Servo_Move(htim2, TIM_CHANNEL_1, outputs[0]);
	Servo_Move(htim2, TIM_CHANNEL_2, outputs[1]);
	Servo_Move(htim2, TIM_CHANNEL_3, outputs[2]);
	Servo_Move(htim2, TIM_CHANNEL_4, outputs[3]);

	Servo_Move(htim3, TIM_CHANNEL_1, outputs[4]);
	Servo_Move(htim3, TIM_CHANNEL_2, outputs[5]);
	Servo_Move(htim3, TIM_CHANNEL_3, outputs[6]);
	Servo_Move(htim3, TIM_CHANNEL_4, outputs[7]);
}



