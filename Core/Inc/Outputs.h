/*
 * Outputs.h
 *
 *  Created on: Sep 1, 2025
 *      Author: sergi
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "stm32f7xx_hal.h"
#include "Parameters.h"
#include "Flight_Modes.h"

HAL_StatusTypeDef Output_Begin(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3);

void Servo_Move(TIM_HandleTypeDef *timerHandle, int channel, float pulse_width_us);

void Output_Update(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3);


#endif /* INC_OUTPUTS_H_ */
