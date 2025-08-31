/*
 * Quaternion.h
 *
 *  Created on: Aug 25, 2025
 *      Author: sergi
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include <math.h>

typedef struct{
	float w;
	float x;
	float y;
	float z;
} Quaternion;

#define DEG_TO_RAD 0.01745329252
#define RAD_TO_DEG 57.29577951

Quaternion Quaternion_mult(Quaternion q1, Quaternion q2);

float Quaternion_dot(Quaternion q1, Quaternion q2);

Quaternion Quaternion_scalar(Quaternion q1, float a);

Quaternion Quaternion_add(Quaternion q1, Quaternion q2);

Quaternion Quaternion_normalize(Quaternion q1);

void Quaternion_to_Euler(Quaternion q, float *roll, float *pitch, float *yaw);

Quaternion Euler_to_Quaternion(float roll, float pitch, float yaw);

Quaternion Quaternion_Slerp(Quaternion q1, Quaternion q2, float gain);

Quaternion Quaternion_Lerp(Quaternion q1, Quaternion q2, float gain);

#endif /* INC_QUATERNION_H_ */
