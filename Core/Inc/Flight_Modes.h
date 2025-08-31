/*
 * Flight_Modes.h
 *
 *  Created on: Aug 29, 2025
 *      Author: sergi
 */

#ifndef INC_FLIGHT_MODES_H_
#define INC_FLIGHT_MODES_H_

#include <math.h>
#include "Sbus.h"
#include "ICM42688P.h"

#define STABILIZED_MAX_ROLL M_PI_4
#define STABILIZED_MAX_PITCH M_PI_4
#define STABILIZED_YAW_RATE 0.01

#define ACRO_ROLL_RATE 0.05
#define ACRO_PITCH_RATE 0.05
#define ACRO_YAW_RATE 0.05

typedef enum{
	MANUAL_MODE,
	STABILIZED_MODE,
	ACRO_MODE
}Flight_Mode;

void Stabilized_Mode(Sbus channels);

void Acro_Mode(Sbus channels);

void Manual_Mode(Sbus channels, float *outputs);


#endif /* INC_FLIGHT_MODES_H_ */
