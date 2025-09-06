/*
 * Outputs.c
 *
 *  Created on: Sep 1, 2025
 *      Author: sergi
 */

#include "Outputs.h"

uint16_t outputs[8];
float functions[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
extern Arming arming;
extern ParameterTable parameters;

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

	//Clamp to [-1,1] range
	for(int i = 0; i < 10; i++) {
		if(functions[i]<-1.0f) functions[i] = -1.0f;
		if(functions[i]>1.0f) functions[i] = 1.0f;
	}

	outputs[0] = functions[parameters.OUT1_FUNCTION] * (parameters.OUT1_MAX - parameters.OUT1_MIN)*(-parameters.OUT1_REVERSE+0.5) + parameters.OUT1_TRIM;
	outputs[1] = functions[parameters.OUT2_FUNCTION] * (parameters.OUT2_MAX - parameters.OUT2_MIN)*(-parameters.OUT2_REVERSE+0.5) + parameters.OUT2_TRIM;
	outputs[2] = functions[parameters.OUT3_FUNCTION] * (parameters.OUT3_MAX - parameters.OUT3_MIN)*(-parameters.OUT3_REVERSE+0.5) + parameters.OUT3_TRIM;
	outputs[3] = functions[parameters.OUT4_FUNCTION] * (parameters.OUT4_MAX - parameters.OUT4_MIN)*(-parameters.OUT4_REVERSE+0.5) + parameters.OUT4_TRIM;
	outputs[4] = functions[parameters.OUT5_FUNCTION] * (parameters.OUT5_MAX - parameters.OUT5_MIN)*(-parameters.OUT5_REVERSE+0.5) + parameters.OUT5_TRIM;
	outputs[5] = functions[parameters.OUT6_FUNCTION] * (parameters.OUT6_MAX - parameters.OUT6_MIN)*(-parameters.OUT6_REVERSE+0.5) + parameters.OUT6_TRIM;
	outputs[6] = functions[parameters.OUT7_FUNCTION] * (parameters.OUT7_MAX - parameters.OUT7_MIN)*(-parameters.OUT7_REVERSE+0.5) + parameters.OUT7_TRIM;
	outputs[7] = functions[parameters.OUT8_FUNCTION] * (parameters.OUT8_MAX - parameters.OUT8_MIN)*(-parameters.OUT8_REVERSE+0.5) + parameters.OUT8_TRIM;


	Servo_Move(htim2, TIM_CHANNEL_1, outputs[0]);
	Servo_Move(htim2, TIM_CHANNEL_2, outputs[1]);
	Servo_Move(htim2, TIM_CHANNEL_3, outputs[2]);
	Servo_Move(htim2, TIM_CHANNEL_4, outputs[3]);

	Servo_Move(htim3, TIM_CHANNEL_1, outputs[4]);
	Servo_Move(htim3, TIM_CHANNEL_2, outputs[5]);
	Servo_Move(htim3, TIM_CHANNEL_3, outputs[6]);
	Servo_Move(htim3, TIM_CHANNEL_4, outputs[7]);
}
