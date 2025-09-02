/*
 * AHRS.h
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */

#ifndef INC_AHRS_H_
#define INC_AHRS_H_

#include "ICM42688P.h"
#include <math.h>

void AHRS_Update_Gyro(Vec3 gyro, float dt);

void AHRS_Update_Acc(Vec3 acc);

void AHRS_Update_Complementary_Filter(Vec3 gyro, Vec3 acc, float gain, float dt);

#endif /* INC_AHRS_H_ */
