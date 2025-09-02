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
#include "Outputs.h"
#include "PIDs.h"

#define STABILIZED_MAX_ROLL M_PI_4
#define STABILIZED_MAX_PITCH M_PI_4

#define ACRO_ROLL_RATE 0.02
#define ACRO_PITCH_RATE 0.02

typedef enum{
	MANUAL_MODE,
	STABILIZED_MODE,
	ACRO_MODE
}Flight_Mode;

typedef enum{
	DISARMED,
	ARMED
}Arming;

void Stabilized_Mode(Sbus receiver, float dt);

void Acro_Mode(Sbus receiver, float dt);

void Manual_Mode(Sbus receiver);

void Process_Input(Sbus receiver);


#endif /* INC_FLIGHT_MODES_H_ */
