/*
 * PIDs.c
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */


#include "PIDs.h"

extern float functions[10];

Vec3 ref_prev, error_prev, integral = {0.0f, 0.0f, 0.0f};

void PID_Update(Vec3 ref, Vec3 attitude, float dt){

	Vec3 error = {(ref.x - attitude.x), (ref.y - attitude.y), (ref.z - attitude.z)};

	integral.x += error.x * dt;
	integral.y += error.y * dt;
	integral.z += error.z * dt;


	functions[AILERONS] = ROLL_KP*error.x + ROLL_KD*(error.x-error_prev.x)/dt + ROLL_KI*integral.x;
	functions[ELEVATOR] = PITCH_KP*error.y + PITCH_KD*(error.y-error_prev.y)/dt + PITCH_KI*integral.y;

	functions[AILERONS] /= M_PI_2;
	functions[ELEVATOR] /= M_PI_2;

	//Clamp to [-1,1] range
	for(int i = 0; i < 10; i++) {
		if(functions[i]<-1.0f) functions[i] = -1.0f;
		if(functions[i]>1.0f) functions[i] = 1.0f;
	}

	error_prev = (Vec3) {.x = error.x, .y = error.y, .z = error.z};
}
