/*
 * Outputs.h
 *
 *  Created on: Sep 1, 2025
 *      Author: sergi
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "stm32f7xx_hal.h"
#include "Flight_Modes.h"

typedef enum{
	NONE,
	AILERONS,
	ELEVATOR,
	THROTTLE,
	RUDDER,
	FLAP
}Output_Functions;

#define OUT1_FUNCTION THROTTLE
#define OUT2_FUNCTION AILERONS
#define OUT3_FUNCTION ELEVATOR
#define OUT4_FUNCTION RUDDER
#define OUT5_FUNCTION NONE
#define OUT6_FUNCTION NONE
#define OUT7_FUNCTION NONE
#define OUT8_FUNCTION NONE

#define OUT1_REVERSE 0 //0: Normal, 1: Reverse
#define OUT2_REVERSE 0
#define OUT3_REVERSE 0
#define OUT4_REVERSE 0
#define OUT5_REVERSE 0
#define OUT6_REVERSE 0
#define OUT7_REVERSE 0
#define OUT8_REVERSE 0

#define OUT1_TRIM 1500
#define OUT2_TRIM 1500
#define OUT3_TRIM 1500
#define OUT4_TRIM 1500
#define OUT5_TRIM 1500
#define OUT6_TRIM 1500
#define OUT7_TRIM 1500
#define OUT8_TRIM 1500

#define OUT1_MIN 1000
#define OUT2_MIN 1000
#define OUT3_MIN 1000
#define OUT4_MIN 1000
#define OUT5_MIN 1000
#define OUT6_MIN 1000
#define OUT7_MIN 1000
#define OUT8_MIN 1000

#define OUT1_MAX 2000
#define OUT2_MAX 2000
#define OUT3_MAX 2000
#define OUT4_MAX 2000
#define OUT5_MAX 2000
#define OUT6_MAX 2000
#define OUT7_MAX 2000
#define OUT8_MAX 2000

HAL_StatusTypeDef Output_Begin(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3);

void Servo_Move(TIM_HandleTypeDef *timerHandle, int channel, float pulse_width_us);

void Output_Update(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3);


#endif /* INC_OUTPUTS_H_ */
