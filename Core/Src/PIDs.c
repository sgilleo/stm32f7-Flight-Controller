/*
 * PIDs.c
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */


#include "PIDs.h"

extern float functions[10];

Vec3 ref_prev, error_prev, integral = {0.0f, 0.0f, 0.0f};

void PID_Stabilized_Update(Vec3 ref, Vec3 attitude, float dt){

	Vec3 error = {(ref.x - attitude.x), (ref.y - attitude.y), (ref.z - attitude.z)};

	integral.x += error.x * dt;
	integral.y += error.y * dt;
	integral.z += error.z * dt;


	functions[AILERONS] = ROLL_KP_STABILIZED*error.x + ROLL_KD_STABILIZED*(error.x-error_prev.x)/dt + ROLL_KI_STABILIZED*integral.x;
	functions[ELEVATOR] = PITCH_KP_STABILIZED*error.y + PITCH_KD_STABILIZED*(error.y-error_prev.y)/dt + PITCH_KI_STABILIZED*integral.y;

	functions[AILERONS] /= M_PI_2;
	functions[ELEVATOR] /= M_PI_2;

	error_prev = (Vec3) {.x = error.x, .y = error.y, .z = error.z};
}

void PID_Acro_Update(Vec3 ref, Vec3 rates, float dt){

	Vec3 error = {(ref.x - rates.x), (ref.y - rates.y), (ref.z + rates.z)};

	integral.x += error.x * dt;
	integral.y += error.y * dt;
	integral.z += error.z * dt;


	functions[AILERONS] = ROLL_KP_ACRO*error.x + ROLL_KD_ACRO*(error.x-error_prev.x)/dt + ROLL_KI_ACRO*integral.x;
	functions[ELEVATOR] = PITCH_KP_ACRO*error.y + PITCH_KD_ACRO*(error.y-error_prev.y)/dt + PITCH_KI_ACRO*integral.y;
	functions[RUDDER] = YAW_KP_ACRO*error.z + YAW_KD_ACRO*(error.z-error_prev.z)/dt;

	functions[AILERONS] /= M_PI_2;
	functions[ELEVATOR] /= M_PI_2;
	functions[RUDDER] /= M_PI_2;

	error_prev = (Vec3) {.x = error.x, .y = error.y, .z = error.z};
}
