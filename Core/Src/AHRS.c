/*
 * AHRS.c
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */


#include "AHRS.h"

Vec3 attitude_gyro, attitude_acc, attitude = {0.0f, 0.0f, 0.0f};

void AHRS_Update_Gyro(Vec3 gyro, float dt){

	attitude_gyro.x = attitude.x + gyro.x * dt * DEG_TO_RAD;
	attitude_gyro.y = attitude.y + gyro.y * dt * DEG_TO_RAD;
	attitude_gyro.z = attitude.z + gyro.z * dt * DEG_TO_RAD;

	if(attitude_gyro.x > M_PI) attitude_gyro.x = -attitude_gyro.x;
	if(attitude_gyro.y > M_PI) attitude_gyro.y = -attitude_gyro.y;
	if(attitude_gyro.z > M_PI) attitude_gyro.z = -attitude_gyro.z;

}



void AHRS_Update_Acc(Vec3 accel){

	attitude_acc.x = atan2f(accel.y, accel.z);
	attitude_acc.y = atan2f(-accel.x, sqrt(accel.y*accel.y + accel.z*accel.z));
	attitude_acc.z = attitude_gyro.z;

}

void AHRS_Update_Complementary_Filter(Vec3 gyro, Vec3 acc, float gain, float dt){

	AHRS_Update_Gyro(gyro, dt);
	AHRS_Update_Acc(acc);

	attitude.x = gain*attitude_gyro.x + (1-gain)*attitude_acc.x;
	attitude.y = gain*attitude_gyro.y + (1-gain)*attitude_acc.y;
	attitude.z = gain*attitude_gyro.z + (1-gain)*attitude_acc.z;

}
