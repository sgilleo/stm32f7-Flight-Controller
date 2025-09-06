/*
 * Sbus.h
 *
 *  Created on: Aug 13, 2025
 *      Author: sergi
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "stm32f7xx_hal.h"

typedef enum{
	SBUS_SIGNAL_OK = 0x00,
	SBUS_SIGNAL_LOST = 0x01,
	SBUS_SIGNAL_FAILSAFE = 0x03
} Failsafe;

typedef struct{
	uint8_t buffer[25];
	uint16_t channels[18];
	Failsafe failsafe_status;
	uint8_t dataRdy;
} Sbus;

HAL_StatusTypeDef Sbus_Begin(UART_HandleTypeDef *huart, Sbus *receiver);

void Sbus_decode(Sbus *receiver);

#endif /* INC_SBUS_H_ */
