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

#define ROLL_KP 1.5
#define ROLL_KI 0.0
#define ROLL_KD 0.0

#define PITCH_KP 1.5
#define PITCH_KI 0.0
#define PITCH_KD 0.0

#define YAW_KP 1.5
#define YAW_KI 0.0
#define YAW_KD 0.0

void PID_Update(Vec3 ref, Vec3 attitude, float dt);


#endif /* INC_PIDS_H_ */
