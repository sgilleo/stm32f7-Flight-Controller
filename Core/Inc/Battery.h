/*
 * Battery.h
 *
 *  Created on: Sep 1, 2025
 *      Author: sergi
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include "stm32f7xx_hal.h"

typedef struct{
	uint32_t battery_raw;
	float vbat;
}Battery;

HAL_StatusTypeDef Battery_Begin(ADC_HandleTypeDef *hadc);

void Battery_Update();


#endif /* INC_BATTERY_H_ */
