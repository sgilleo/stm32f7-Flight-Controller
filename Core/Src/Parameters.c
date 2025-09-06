/*
 * Parameters.c
 *
 *  Created on: Sep 6, 2025
 *      Author: sergi
 */

#include "Parameters.h"

#define PARAMS_FLASH_ADDR (0x08060000) //Last Sector Address

const ParameterTable default_parameters = {
		.STABILIZED_MAX_ROLL = 45,
		.STABILIZED_MAX_PITCH = 45,

		.ACRO_ROLL_RATE = 180,
		.ACRO_PITCH_RATE = 180,
		.ACRO_YAW_RATE = 90,

		.ROLL_KD_STABILIZED = 0.6,
		.ROLL_KP_STABILIZED = 1.8,
		.ROLL_KI_STABILIZED = 0.8,

		.PITCH_KD_STABILIZED = 0.6,
		.PITCH_KP_STABILIZED = 1.8,
		.PITCH_KI_STABILIZED = 0.8,

		.ROLL_KD_ACRO = 0.4,
		.ROLL_KP_ACRO = 0.6,
		.ROLL_KI_ACRO = 1.8,

		.PITCH_KD_ACRO = 0.4,
		.PITCH_KP_ACRO = 0.6,
		.PITCH_KI_ACRO = 1.8,

		.YAW_KD_ACRO = 0.4,
		.YAW_KP_ACRO = 0.6,

		.OUT1_FUNCTION = THROTTLE,
		.OUT2_FUNCTION = ELEVATOR,
		.OUT3_FUNCTION = RUDDER,
		.OUT4_FUNCTION = NONE,
		.OUT5_FUNCTION = NONE,
		.OUT6_FUNCTION = NONE,
		.OUT7_FUNCTION = NONE,
		.OUT8_FUNCTION = NONE,

		.OUT1_REVERSE = 0,
		.OUT2_REVERSE = 0,
		.OUT3_REVERSE = 0,
		.OUT4_REVERSE = 0,
		.OUT5_REVERSE = 0,
		.OUT6_REVERSE = 0,
		.OUT7_REVERSE = 0,
		.OUT8_REVERSE = 0,

		.OUT1_TRIM = 1500,
		.OUT2_TRIM = 1500,
		.OUT3_TRIM = 1500,
		.OUT4_TRIM = 1500,
		.OUT5_TRIM = 1500,
		.OUT6_TRIM = 1500,
		.OUT7_TRIM = 1500,
		.OUT8_TRIM = 1500,

		.OUT1_MIN = 1000,
		.OUT2_MIN = 1000,
		.OUT3_MIN = 1000,
		.OUT4_MIN = 1000,
		.OUT5_MIN = 1000,
		.OUT6_MIN = 1000,
		.OUT7_MIN = 1000,
		.OUT8_MIN = 1000,

		.OUT1_MAX = 2000,
		.OUT2_MAX = 2000,
		.OUT3_MAX = 2000,
		.OUT4_MAX = 2000,
		.OUT5_MAX = 2000,
		.OUT6_MAX = 2000,
		.OUT7_MAX = 2000,
		.OUT8_MAX = 2000,

		.THROTTLE_CHANNEL = 1-1,
		.ROLL_CHANNEL = 2-1,
		.PITCH_CHANNEL = 3-1,
		.YAW_CHANNEL = 4-1,
		.ARMING_CHANNEL = 5-1,
		.FLIGHT_MODE_CHANNEL = 6-1
};

ParameterTable parameters;



uint8_t flash_empty(uint32_t addr, size_t size) {
	uint32_t *p = (uint32_t*)addr;
	for (size_t i = 0; i < size/4; i++) {
		if (p[i] != 0xFFFFFFFF) {
			return 0; // at least one word programmed
		}
	}
	return 1; // all erased
}

//Returns error if memory is empty
HAL_StatusTypeDef Load_Parameters(ParameterTable *parameters){
	ParameterTable *flashParams = (ParameterTable*)PARAMS_FLASH_ADDR;

	if(flash_empty((uint32_t)flashParams, sizeof(ParameterTable))){
		return HAL_ERROR;
	}

	memcpy(parameters, flashParams, sizeof(ParameterTable));

	return HAL_OK;

}

HAL_StatusTypeDef Save_Parameters(ParameterTable *parameters){
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef flashErase;
	uint32_t sectorError;

	status = HAL_FLASH_Unlock();
	if(status != HAL_OK) return status;

	flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
	flashErase.Sector = FLASH_SECTOR_7;
	flashErase.NbSectors = 1;
	flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	if(HAL_FLASHEx_Erase(&flashErase, &sectorError) != HAL_OK) return HAL_FLASH_GetError();


	uint32_t *src = (uint32_t*)parameters;
	uint32_t addr = PARAMS_FLASH_ADDR;
	for(int i = 0; i < sizeof(ParameterTable)/4; i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
		if(status != HAL_OK) return status;
		addr += 4;
	}

	status = HAL_FLASH_Lock();
	if(status != HAL_OK) return status;

	return status;
}





