#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "estimator.h"
#include "crsf.h"

typedef enum {
    STATE_DISARMED = 0,
    STATE_LAUNCH_HOVER,
    STATE_WAYPOINT_NAV,
    STATE_LANDING,
    STATE_PARACHUTE
} FlightState_t;

void FlightControl_Init(void);
void FlightControl_Update200Hz(float dt, const VehicleState_t *state, const RC_Input_t *rc,
                               uint32_t now_ms, uint16_t *esc_pwm, uint16_t *servo_x_pwm,
                               uint16_t *servo_y_pwm, uint16_t *chute_pwm);

#endif