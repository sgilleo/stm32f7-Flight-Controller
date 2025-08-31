/*
 * Quaternion.c
 *
 *  Created on: Aug 25, 2025
 *      Author: sergi
 */


#include "Quaternion.h"


Quaternion Quaternion_mult(Quaternion q1, Quaternion q2){
	Quaternion q;
	q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
	q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
	q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
	q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x +	q1.z*q2.w;
	return q;
}

float Quaternion_dot(Quaternion q1, Quaternion q2){
	return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
}

Quaternion Quaternion_scalar(Quaternion q1, float a){
	Quaternion q;
	q.w = a * q1.w;
	q.x = a * q1.x;
	q.y = a * q1.y;
	q.z = a * q1.z;
	return q;
}

Quaternion Quaternion_add(Quaternion q1, Quaternion q2){
	Quaternion q;
	q.w = q1.w + q2.w;
	q.x = q1.x + q2.x;
	q.y = q1.y + q2.y;
	q.z = q1.z + q2.z;
	return q;
}

Quaternion Quaternion_normalize(Quaternion q1){
	Quaternion q;
	float n = sqrt(q1.w*q1.w + q1.x*q1.x + q1.y*q1.y + q1.z*q1.z);
	q.w = q1.w / n;
	q.x = q1.x / n;
	q.y = q1.y / n;
	q.z = q1.z / n;
	return q;
}

void Quaternion_to_Euler(Quaternion q, float *roll, float *pitch, float *yaw)
{
    // normalizar cuaternión
    q = Quaternion_normalize(q);

    // roll (x-axis rotation)
    float sinr_cosp = 2.0 * (q.w*q.x + q.y*q.z);
    float cosr_cosp = 1.0 - 2.0 * (q.x*q.x + q.y*q.y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0 * (q.w*q.y - q.z*q.x);
    if (fabs(sinp) >= 1.0)
        *pitch = copysign(M_PI / 2.0, sinp); // clamp a 90°
    else
        *pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0 * (q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

Quaternion Euler_to_Quaternion(float roll, float pitch, float yaw){

	Quaternion qyaw = {.w = cosf(yaw/2), .x = 0.0f, .y = 0.0f, .z = sinf(yaw/2)};
	Quaternion qpitch = {.w = cosf(pitch/2), .x = 0.0f, .y = sinf(pitch/2), .z = 0.0f};
	Quaternion qroll = {.w = cosf(roll/2), .x = sinf(roll/2), .y = 0.0f, .z = 0.0f};

	return Quaternion_mult(Quaternion_mult(qyaw, qpitch), qroll);
}

Quaternion Quaternion_Slerp(Quaternion q1, Quaternion q2, float gain){
	float theta = acosf(Quaternion_dot(q1, q2));

	if(theta = 0){
		return Quaternion_Lerp(q1, q2, gain);
	}

	float a = sinf((1-gain)*theta)/sinf(theta);
	float b = sinf(gain*theta)/sinf(theta);

	Quaternion q;

	q.w = a*q1.w + b*q2.w;
	q.x = a*q1.x + b*q2.x;
	q.y = a*q1.y + b*q2.y;
	q.z = a*q1.z + b*q2.z;

	return q;

}

Quaternion Quaternion_Lerp(Quaternion q1, Quaternion q2, float gain){
	Quaternion q;

	q.w = (1-gain)*q1.w + gain*q2.w;
	q.x = (1-gain)*q1.x + gain*q2.x;
	q.y = (1-gain)*q1.y + gain*q2.y;
	q.z = (1-gain)*q1.z + gain*q2.z;

	return q;
}


