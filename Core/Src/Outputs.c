/*
 * Outputs.c
 *
 *  Created on: Sep 1, 2025
 *      Author: sergi
 */

#include "Outputs.h"

float outputs[8];
float functions[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
extern Arming arming;

HAL_StatusTypeDef Output_Begin(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3){
	HAL_StatusTypeDef status;

	status = HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
	if(status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
	if(status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
	if(status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);
	if(status != HAL_OK) return status;

	status = HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
	if(status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
	if(status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
	if(status != HAL_OK) return status;
	status = HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_4);
	if(status != HAL_OK) return status;

	return status;
}

void Servo_Move(TIM_HandleTypeDef *timerHandle, int channel, float pulse_width_us){
	uint16_t compare = pulse_width_us * 1.8f;
	__HAL_TIM_SET_COMPARE(timerHandle, channel, compare);

}

void Output_Update(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3){

	if(arming == DISARMED) functions[THROTTLE] = -1.0f;

	outputs[0] = functions[OUT1_FUNCTION] * (OUT1_MAX - OUT1_MIN)*(-OUT1_REVERSE+0.5) + OUT1_TRIM;
	outputs[1] = functions[OUT2_FUNCTION] * (OUT2_MAX - OUT2_MIN)*(-OUT2_REVERSE+0.5) + OUT2_TRIM;
	outputs[2] = functions[OUT3_FUNCTION] * (OUT3_MAX - OUT3_MIN)*(-OUT3_REVERSE+0.5) + OUT3_TRIM;
	outputs[3] = functions[OUT4_FUNCTION] * (OUT4_MAX - OUT4_MIN)*(-OUT4_REVERSE+0.5) + OUT4_TRIM;
	outputs[4] = functions[OUT5_FUNCTION] * (OUT5_MAX - OUT5_MIN)*(-OUT5_REVERSE+0.5) + OUT5_TRIM;
	outputs[5] = functions[OUT6_FUNCTION] * (OUT6_MAX - OUT6_MIN)*(-OUT6_REVERSE+0.5) + OUT6_TRIM;
	outputs[6] = functions[OUT7_FUNCTION] * (OUT7_MAX - OUT7_MIN)*(-OUT7_REVERSE+0.5) + OUT7_TRIM;
	outputs[7] = functions[OUT8_FUNCTION] * (OUT8_MAX - OUT8_MIN)*(-OUT8_REVERSE+0.5) + OUT8_TRIM;


	Servo_Move(htim2, TIM_CHANNEL_1, outputs[0]);
	Servo_Move(htim2, TIM_CHANNEL_2, outputs[1]);
	Servo_Move(htim2, TIM_CHANNEL_3, outputs[2]);
	Servo_Move(htim2, TIM_CHANNEL_4, outputs[3]);

	Servo_Move(htim3, TIM_CHANNEL_1, outputs[4]);
	Servo_Move(htim3, TIM_CHANNEL_2, outputs[5]);
	Servo_Move(htim3, TIM_CHANNEL_3, outputs[6]);
	Servo_Move(htim3, TIM_CHANNEL_4, outputs[7]);
}
