/*
 * Flight_Modes.c
 *
 *  Created on: Aug 29, 2025
 *      Author: sergi
 */

#include "Flight_Modes.h"

Vec3 ref = {0.0f, 0.0f, 0.0f};

void Stabilized_Mode(Sbus channels){
	ref.x = (channels.channels[1]-1500) * 0.002f * STABILIZED_MAX_ROLL;
	ref.y = (channels.channels[2]-1500) * 0.002f * STABILIZED_MAX_PITCH;
	ref.z += (channels.channels[3]-1500) * 0.0031415 * STABILIZED_YAW_RATE;
	if(ref.z >= 2*M_PI) ref.z -= 2*M_PI;
	if(ref.z < 0) ref.z += 2*M_PI;

}

void Acro_Mode(Sbus channels){
	ref.x += (channels.channels[1]-1500) * 0.0031415 * ACRO_ROLL_RATE;
	ref.y += (channels.channels[2]-1500) * 0.0031415 * ACRO_PITCH_RATE;
	ref.z += (channels.channels[3]-1500) * 0.0031415 * ACRO_YAW_RATE;
	if(ref.z >= 2*M_PI) ref.z -= 2*M_PI;
	if(ref.z < 0) ref.z += 2*M_PI;
}

void Manual_Mode(Sbus channels, float *outputs){
	outputs[0] = channels.channels[1];
	outputs[1] = channels.channels[2];
	outputs[2] = channels.channels[3];
}
