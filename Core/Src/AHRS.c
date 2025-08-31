/*
 * AHRS.c
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */


#include "AHRS.h"

//Quaternion q, q_gyro, q_acc;
Vec3 attitude_gyro, attitude_acc, attitude;

void AHRS_Init(){

	//q = (Quaternion) {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
	//q_gyro = (Quaternion) {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
	//q_acc = (Quaternion) {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};

	attitude_gyro = (Vec3) {.x = 0.0f, .y = 0.0f, .z = 0.0f};
	attitude_acc = (Vec3) {.x = 0.0f, .y = 0.0f, .z = 0.0f};
	attitude = (Vec3) {.x = 0.0f, .y = 0.0f, .z = 0.0f};
}

void AHRS_Update_Gyro(Vec3 gyro, float dt){

	/*Quaternion qDot;

	qDot.w = (-q.x*gyro.x - q.y*gyro.y - q.z*gyro.z) * 0.5f; //q_gyro x w
	qDot.x = ( q.w*gyro.x + q.y*gyro.z - q.z*gyro.y) * 0.5f;
	qDot.y = ( q.w*gyro.y - q.x*gyro.z + q.z*gyro.x) * 0.5f;
	qDot.z = ( q.w*gyro.z + q.x*gyro.y - q.y*gyro.x) * 0.5f;

	q_gyro.w = q.w + qDot.w * dt * DEG_TO_RAD;
	q_gyro.x = q.x + qDot.x * dt * DEG_TO_RAD;
	q_gyro.y = q.y + qDot.y * dt * DEG_TO_RAD;
	q_gyro.z = q.z + qDot.z * dt * DEG_TO_RAD;

	q_gyro = Quaternion_normalize(q_gyro);*/

	attitude_gyro.x = attitude.x + gyro.x * dt * DEG_TO_RAD;
	attitude_gyro.y = attitude.y + gyro.y * dt * DEG_TO_RAD;
	attitude_gyro.z = attitude.z + gyro.z * dt * DEG_TO_RAD;

}



void AHRS_Update_Acc(Vec3 accel){

	attitude_acc.x = atan2f(accel.y, accel.z);
	attitude_acc.y = atan2f(-accel.x, sqrt(accel.y*accel.y + accel.z*accel.z));
	attitude_acc.z = attitude_gyro.z;

}

void AHRS_Update_Complementary_Filter(float gain){

	//q = Quaternion_Slerp(q_acc, q_gyro, 0.80);
	//q = Quaternion_Lerp(q_acc, q_gyro, 0.95);
	//q = Quaternion_normalize(q);

	attitude.x = gain*attitude_gyro.x + (1-gain)*attitude_acc.x;
	attitude.y = gain*attitude_gyro.y + (1-gain)*attitude_acc.y;
	attitude.z = gain*attitude_gyro.z + (1-gain)*attitude_acc.z;

}
