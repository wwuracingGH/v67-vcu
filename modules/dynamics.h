/* 
 * Nicole Swierstra
 * ======= Vehicle Dynamics Modeling ======== 
 * Coming soon, maybe 
 */

#ifndef _MODULES_DYNAMICS_H_
#define _MODULES_DYNAMICS_H_

#include "control.h"

/* TODO */
typedef struct {
    uint32_t dynamicParams;
} VehicleDynamicParams_t;

/* TODO */
typedef struct {
    uint32_t dynamicState;
} VehicleDynamicState_t;

/* TODO */
typedef struct {
    uint32_t TODO;
} ObserverDynamicState_t;
/*
VehicleDynamicState_t* updatedModel(uint32_t newTime);

void applyControl(ControlReq_t* newControl, uint32_t newTime);

void applyModel(VehicleDynamicState_t* newState);
void applyObserver();
*/
#endif