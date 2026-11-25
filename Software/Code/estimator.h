#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "sensors.h"

typedef struct {
    float q[4];
    float euler[3];
    float pos[3];
    float vel[3];
} VehicleState_t;

void Estimator_Init(VehicleState_t *state);
void Estimator_Update(VehicleState_t *state, const SensorData_t *sens, float dt);

#endif