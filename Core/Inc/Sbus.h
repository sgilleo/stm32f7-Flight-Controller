/*
 * Sbus.h
 *
 *  Created on: Aug 13, 2025
 *      Author: sergi
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "stm32f7xx_hal.h"

#define THROTTLE_CHANNEL 1-1
#define ROLL_CHANNEL 2-1
#define PITCH_CHANNEL 3-1
#define YAW_CHANNEL 4-1
#define ARMING_CHANNEL 5-1
#define FLIGHT_MODE_CHANNEL 6-1

typedef enum{
	SBUS_SIGNAL_OK = 0x00,
	SBUS_SIGNAL_LOST = 0x01,
	SBUS_SIGNAL_FAILSAFE = 0x03
} Failsafe;

typedef struct{
	uint8_t buffer[25];
	float channels[18];
	Failsafe failsafe_status;
	uint8_t dataRdy;
} Sbus;

HAL_StatusTypeDef Sbus_Begin(UART_HandleTypeDef *huart, Sbus *receiver);

void Sbus_decode(Sbus *receiver);

#endif /* INC_SBUS_H_ */
