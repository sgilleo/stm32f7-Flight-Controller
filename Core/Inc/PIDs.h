/*
 * PIDs.h
 *
 *  Created on: Aug 30, 2025
 *      Author: sergi
 */

#ifndef INC_PIDS_H_
#define INC_PIDS_H_

#include "ICM42688P.h"
#include <math.h>
#include "Outputs.h"

#define ROLL_KD_STABILIZED 0.6
#define ROLL_KP_STABILIZED 1.8
#define ROLL_KI_STABILIZED 0.8

#define PITCH_KD_STABILIZED 0.6
#define PITCH_KP_STABILIZED 1.8
#define PITCH_KI_STABILIZED 0.8

#define YAW_KD_STABILIZED 0.6
#define YAW_KP_STABILIZED 1.8
#define YAW_KI_STABILIZED 0.8

#define ROLL_KD_ACRO 0.0
#define ROLL_KP_ACRO 0.6
#define ROLL_KI_ACRO 1.8

#define PITCH_KD_ACRO 0.0
#define PITCH_KP_ACRO 0.6
#define PITCH_KI_ACRO 1.8

#define YAW_KD_ACRO 0.0
#define YAW_KP_ACRO 0.6 //No KI: Yaw only try to stop rotation


void PID_Stabilized_Update(Vec3 ref, Vec3 attitude, float dt);

void PID_Acro_Update(Vec3 ref, Vec3 rates, float dt);


#endif /* INC_PIDS_H_ */
