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

void PID_Stabilized_Update(Vec3 ref, Vec3 attitude, float dt);

void PID_Acro_Update(Vec3 ref, Vec3 rates, float dt);


#endif /* INC_PIDS_H_ */
