/*
 * Battery.c
 *
 *  Created on: Sep 1, 2025
 *      Author: sergi
 */


#include "Battery.h"

Battery battery;

HAL_StatusTypeDef Battery_Begin(ADC_HandleTypeDef *hadc){
	HAL_StatusTypeDef status = HAL_ADC_Start_DMA(hadc, &battery.battery_raw, 1);
	if(status != HAL_OK) return status;

	return status;
}

void Battery_Update(){
	battery.vbat = 0.00886230468f * battery.battery_raw; // 3.3/4096*11
}
