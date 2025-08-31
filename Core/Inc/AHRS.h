/*
 * AHRS.h
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */

#ifndef INC_AHRS_H_
#define INC_AHRS_H_

#include "Quaternion.h"
#include "ICM42688P.h"

void AHRS_Init();

void AHRS_Update_Gyro(Vec3 gyro, float dt);

void AHRS_Update_Acc(Vec3 acc);

void AHRS_Update_Complementary_Filter(float gain);

#endif /* INC_AHRS_H_ */
