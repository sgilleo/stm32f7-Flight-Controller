/*
 * Flight_Modes.c
 *
 *  Created on: Aug 29, 2025
 *      Author: sergi
 */

#include "Flight_Modes.h"

Vec3 ref = {0.0f, 0.0f, 0.0f};
Flight_Mode flight_mode = MANUAL_MODE;
Flight_Mode prev_flight_mode = MANUAL_MODE; //For detecting edges in flight modes
Arming arming = DISARMED;
uint8_t arming_edge_detect = 0;


extern Vec3 attitude, integral;
extern float functions[10];
extern ICM42688 imu;


void Stabilized_Mode(Sbus receiver, float dt){
	//In Stabilized Mode Reference is the angle (radians)
	ref.x = (receiver.channels[ROLL_CHANNEL]-1500) * 0.002f * STABILIZED_MAX_ROLL * DEG_TO_RAD;
	ref.y = (receiver.channels[PITCH_CHANNEL]-1500) * 0.002f * STABILIZED_MAX_PITCH * DEG_TO_RAD;

	PID_Stabilized_Update(ref, attitude, dt);

	functions[RUDDER] = (receiver.channels[YAW_CHANNEL]-1500.0f)/500.0f;
	functions[THROTTLE] = (receiver.channels[THROTTLE_CHANNEL]-1500.0f)/500.0f;

}

void Acro_Mode(Sbus receiver, float dt){
	//In Stabilized Mode Reference is the angular velocity (radians/second)
	ref.x = (receiver.channels[ROLL_CHANNEL]-1500) * 0.002f * ACRO_ROLL_RATE * DEG_TO_RAD;
	ref.y = (receiver.channels[PITCH_CHANNEL]-1500) * 0.002f * ACRO_PITCH_RATE * DEG_TO_RAD;
	ref.z = (receiver.channels[YAW_CHANNEL]-1500) * 0.002f * ACRO_YAW_RATE * DEG_TO_RAD;

	PID_Acro_Update(ref, imu.gyro, dt);

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

	//DISARM ON FAILSAFE
	if(receiver.failsafe_status == SBUS_SIGNAL_FAILSAFE){
		flight_mode = STABILIZED_MODE;
		arming = DISARMED;
		functions[THROTTLE] = -1.0f;
	}

	if(prev_flight_mode != flight_mode) { //Change in flight mode
		integral = (Vec3){0.0f, 0.0f, 0.0f}; //Reset integral part of PID
	}


	//Set Arming

	if(arming == DISARMED &&
		receiver.channels[ARMING_CHANNEL] > 1500 &&
		arming_edge_detect == 1 &&
		receiver.channels[THROTTLE_CHANNEL] < 1050 ) arming = ARMED;
	if(arming == ARMED && receiver.channels[ARMING_CHANNEL] <= 1500) arming = DISARMED;


	arming_edge_detect = (receiver.channels[ARMING_CHANNEL] <= 1500)? 1:0;
	prev_flight_mode = flight_mode;
}


