/*
 * PIDs.c
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */


#include "PIDs.h"

extern float functions[10];

Vec3 ref_prev, filtered_error, filtered_error_prev, integral = {0.0f, 0.0f, 0.0f};

void PID_Stabilized_Update(Vec3 ref, Vec3 attitude, float dt){

	Vec3 error = {(ref.x - attitude.x), (ref.y - attitude.y), (ref.z - attitude.z)};

	//Low Pass Filter for derivative of error (rate)
	float tau = 1/(2*M_PI*2);
	float alpha = tau/(tau+dt);

	filtered_error.x = (1-alpha) * error.x + alpha * filtered_error_prev.x;
	filtered_error.y = (1-alpha) * error.y + alpha * filtered_error_prev.y;
	filtered_error.z = (1-alpha) * error.z + alpha * filtered_error_prev.z;

	integral.x += error.x * dt;
	integral.y += error.y * dt;
	integral.z += error.z * dt;


	functions[AILERONS] = ROLL_KP_STABILIZED*error.x + ROLL_KD_STABILIZED*(filtered_error.x-filtered_error_prev.x)/dt + ROLL_KI_STABILIZED*integral.x;
	functions[ELEVATOR] = PITCH_KP_STABILIZED*error.y + PITCH_KD_STABILIZED*(filtered_error.y-filtered_error_prev.y)/dt + PITCH_KI_STABILIZED*integral.y;

	functions[AILERONS] /= M_PI_2;
	functions[ELEVATOR] /= M_PI_2;

	filtered_error_prev = (Vec3) {filtered_error.x, filtered_error.y, filtered_error.z};
}

void PID_Acro_Update(Vec3 ref, Vec3 rates, float dt){

	Vec3 error = {(ref.x - rates.x), (ref.y - rates.y), (ref.z + rates.z)};

	//Low Pass Filter for derivative of error (rate)
	float tau = 1/(2*M_PI*0.4);
	float alpha = tau/(tau+dt);

	filtered_error.x = (1-alpha) * error.x + alpha * filtered_error_prev.x;
	filtered_error.y = (1-alpha) * error.y + alpha * filtered_error_prev.y;
	filtered_error.z = (1-alpha) * error.z + alpha * filtered_error_prev.z;

	integral.x += error.x * dt;
	integral.y += error.y * dt;
	integral.z += error.z * dt;


	functions[AILERONS] = ROLL_KP_ACRO*error.x + ROLL_KD_ACRO*(filtered_error.x-filtered_error_prev.x)/dt + ROLL_KI_ACRO*integral.x;
	functions[ELEVATOR] = PITCH_KP_ACRO*error.y + PITCH_KD_ACRO*(filtered_error.y-filtered_error_prev.y)/dt + PITCH_KI_ACRO*integral.y;
	functions[RUDDER] = YAW_KP_ACRO*error.z + YAW_KD_ACRO*(filtered_error.z-filtered_error_prev.z)/dt;

	functions[AILERONS] /= M_PI_2;
	functions[ELEVATOR] /= M_PI_2;
	functions[RUDDER] /= M_PI_2;

	filtered_error_prev = (Vec3) {filtered_error.x, filtered_error.y, filtered_error.z};
}
