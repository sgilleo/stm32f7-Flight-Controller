/*
 * Flight_Modes.c
 *
 *  Created on: Aug 29, 2025
 *      Author: sergi
 */

#include "Flight_Modes.h"

Vec3 ref = {0.0f, 0.0f, 0.0f};
Flight_Mode flight_mode = MANUAL_MODE;
Arming arming = DISARMED;
uint8_t arming_edge_detect = 0;

extern Vec3 attitude;
extern float functions[10];


void Stabilized_Mode(Sbus receiver, float dt){
	ref.x = (receiver.channels[ROLL_CHANNEL]-1500) * 0.002f * STABILIZED_MAX_ROLL;
	ref.y = (receiver.channels[PITCH_CHANNEL]-1500) * 0.002f * STABILIZED_MAX_PITCH;

	PID_Update(ref, attitude, dt);

	functions[RUDDER] = (receiver.channels[YAW_CHANNEL]-1500.0f)/500.0f;
	functions[THROTTLE] = (receiver.channels[THROTTLE_CHANNEL]-1500.0f)/500.0f;

}

void Acro_Mode(Sbus receiver, float dt){
	ref.x += (receiver.channels[ROLL_CHANNEL]-1500) * 0.0031415 * ACRO_ROLL_RATE;
	ref.y += (receiver.channels[PITCH_CHANNEL]-1500) * 0.0031415 * ACRO_PITCH_RATE;

	if(ref.x > M_PI || ref.x <= -M_PI) ref.x = -ref.x;
	if(ref.y > M_PI || ref.y <= -M_PI) ref.y = -ref.y;

	PID_Update(ref, attitude, dt);

	functions[RUDDER] = (receiver.channels[YAW_CHANNEL]-1500.0f)/500.0f;
	functions[THROTTLE] = (receiver.channels[THROTTLE_CHANNEL]-1500.0f)/500.0f;

}

void Manual_Mode(Sbus receiver){
	functions[THROTTLE] = (receiver.channels[THROTTLE_CHANNEL]-1500.0f)/500.0f;
	functions[AILERONS] = (receiver.channels[ROLL_CHANNEL]-1500.0f)/500.0f;
	functions[ELEVATOR] = (receiver.channels[PITCH_CHANNEL]-1500.0f)/500.0f;
	functions[RUDDER] = (receiver.channels[YAW_CHANNEL]-1500.0f)/500.0f;

}

void Process_Input(Sbus receiver){

	//Configure MODES
	if(receiver.channels[FLIGHT_MODE_CHANNEL] < 1200){
		flight_mode = MANUAL_MODE;
	}
	else if (receiver.channels[FLIGHT_MODE_CHANNEL] >= 1200 &&
			receiver.channels[FLIGHT_MODE_CHANNEL] < 1800){
		flight_mode = STABILIZED_MODE;
	}
	else{
		flight_mode = ACRO_MODE;
	}



	//Set Arming

	if(arming == DISARMED &&
		receiver.channels[ARMING_CHANNEL] > 1500 &&
		arming_edge_detect == 1 &&
		receiver.channels[THROTTLE_CHANNEL] < 1050 ) arming = ARMED;
	if(arming == ARMED && receiver.channels[ARMING_CHANNEL] <= 1500) arming = DISARMED;


	arming_edge_detect = (receiver.channels[ARMING_CHANNEL] <= 1500)? 1:0;
}


