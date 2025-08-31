/*
 * PIDs.c
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */


#include "PIDs.h"

extern float outputs[8];

Vec3 ref_prev, error_prev, integral = {0.0f, 0.0f, 0.0f};

void PID_Update(Vec3 ref, Vec3 attitude, float dt){

	Vec3 error = {(ref.x - attitude.x), (ref.y - attitude.y), (ref.z - attitude.z)};

	integral.x += error.x * dt;
	integral.y += error.y * dt;
	integral.z += error.z * dt;


	outputs[0] = ROLL_KP*error.x + ROLL_KD*(error.x-error_prev.x)/dt + ROLL_KI*integral.x;
	outputs[1] = PITCH_KP*error.y + PITCH_KD*(error.y-error_prev.y)/dt + PITCH_KI*integral.y;
	outputs[2] = YAW_KP*error.z + YAW_KD*(error.z-error_prev.z)/dt + YAW_KI*integral.z;

	outputs[0] = outputs[0]* 500/M_PI_2 + 1500;
	outputs[1] = outputs[1]* 500/M_PI_2 + 1500;
	outputs[2] = outputs[2]* 500/M_PI_2 + 1500;

	error_prev = (Vec3) {.x = error.x, .y = error.y, .z = error.z};
}
