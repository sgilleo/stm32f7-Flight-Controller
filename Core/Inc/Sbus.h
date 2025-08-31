/*
 * Sbus.h
 *
 *  Created on: Aug 13, 2025
 *      Author: sergi
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "stm32f7xx_hal.h"

typedef struct{
	uint8_t buffer[25];
	float channels[18];
	uint8_t dataRdy;
	uint8_t inSync;
	uint8_t counter;
	uint8_t firstByte;
} Sbus;

HAL_StatusTypeDef Sbus_Begin(UART_HandleTypeDef *huart, Sbus *receiver);

void Sbus_decode(uint8_t *buffer, float *channels);

#endif /* INC_SBUS_H_ */
