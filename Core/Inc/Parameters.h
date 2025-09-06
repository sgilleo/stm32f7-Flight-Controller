/*
 * Parameters.h
 *
 *  Created on: Sep 6, 2025
 *      Author: sergi
 */

#ifndef INC_PARAMETERS_H_
#define INC_PARAMETERS_H_

#include "stm32f7xx_hal.h"
#include "Outputs.h"
#include "string.h"

typedef enum{
	NONE,
	AILERONS,
	ELEVATOR,
	THROTTLE,
	RUDDER,
	FLAP
}Output_Functions;

typedef struct{

	float STABILIZED_MAX_ROLL; //Degrees
	float STABILIZED_MAX_PITCH;

	float ACRO_ROLL_RATE; //Full Stick Left/Right: Degrees / Second
	float ACRO_PITCH_RATE;
	float ACRO_YAW_RATE;

	float ROLL_KD_STABILIZED;
	float ROLL_KP_STABILIZED;
	float ROLL_KI_STABILIZED;

	float PITCH_KD_STABILIZED;
	float PITCH_KP_STABILIZED;
	float PITCH_KI_STABILIZED;

	float ROLL_KD_ACRO;
	float ROLL_KP_ACRO;
	float ROLL_KI_ACRO;

	float PITCH_KD_ACRO;
	float PITCH_KP_ACRO;
	float PITCH_KI_ACRO;

	float YAW_KD_ACRO;
	float YAW_KP_ACRO; //No KI: Yaw only try to stop rotation

	Output_Functions OUT1_FUNCTION;
	Output_Functions OUT2_FUNCTION;
	Output_Functions OUT3_FUNCTION;
	Output_Functions OUT4_FUNCTION;
	Output_Functions OUT5_FUNCTION;
	Output_Functions OUT6_FUNCTION;
	Output_Functions OUT7_FUNCTION;
	Output_Functions OUT8_FUNCTION;

	uint8_t OUT1_REVERSE; //0: Normal, 1: Reverse
	uint8_t OUT2_REVERSE;
	uint8_t OUT3_REVERSE;
	uint8_t OUT4_REVERSE;
	uint8_t OUT5_REVERSE;
	uint8_t OUT6_REVERSE;
	uint8_t OUT7_REVERSE;
	uint8_t OUT8_REVERSE;

	uint16_t OUT1_TRIM;
	uint16_t OUT2_TRIM;
	uint16_t OUT3_TRIM;
	uint16_t OUT4_TRIM;
	uint16_t OUT5_TRIM;
	uint16_t OUT6_TRIM;
	uint16_t OUT7_TRIM;
	uint16_t OUT8_TRIM;

	uint16_t OUT1_MIN;
	uint16_t OUT2_MIN;
	uint16_t OUT3_MIN;
	uint16_t OUT4_MIN;
	uint16_t OUT5_MIN;
	uint16_t OUT6_MIN;
	uint16_t OUT7_MIN;
	uint16_t OUT8_MIN;

	uint16_t OUT1_MAX;
	uint16_t OUT2_MAX;
	uint16_t OUT3_MAX;
	uint16_t OUT4_MAX;
	uint16_t OUT5_MAX;
	uint16_t OUT6_MAX;
	uint16_t OUT7_MAX;
	uint16_t OUT8_MAX;

	uint8_t THROTTLE_CHANNEL;
	uint8_t ROLL_CHANNEL;
	uint8_t PITCH_CHANNEL;
	uint8_t YAW_CHANNEL;
	uint8_t ARMING_CHANNEL;
	uint8_t FLIGHT_MODE_CHANNEL;

}ParameterTable;

HAL_StatusTypeDef Load_Parameters(ParameterTable *parameters);

HAL_StatusTypeDef Save_Parameters(ParameterTable *parameters);



#endif /* INC_PARAMETERS_H_ */
