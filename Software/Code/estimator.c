#include "estimator.h"
#include <math.h>

void Estimator_Init(VehicleState_t *state) {
    state->q[0] = 1.0f;
    state->q[1] = 0.0f;
    state->q[2] = 0.0f;
    state->q[3] = 0.0f;
    for (int i = 0; i < 3; i++) {
        state->euler[i] = 0.0f;
        state->pos[i] = 0.0f;
        state->vel[i] = 0.0f;
    }
}

void Estimator_Update(VehicleState_t *state, const SensorData_t *sens, float dt) {
    float gx = sens->gyro[0], gy = sens->gyro[1], gz = sens->gyro[2];
    float ax = sens->accel[0], ay = sens->accel[1], az = sens->accel[2];
    float q0 = state->q[0], q1 = state->q[1], q2 = state->q[2], q3 = state->q[3];

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.001f) {
        ax /= norm; ay /= norm; az /= norm;

        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float s_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (s_norm > 0.0f) {
            s0 /= s_norm; s1 /= s_norm; s2 /= s_norm; s3 /= s_norm;
        }

        const float beta = 0.04f;
        float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        state->q[0] = q0 / q_norm;
        state->q[1] = q1 / q_norm;
        state->q[2] = q2 / q_norm;
        state->q[3] = q3 / q_norm;
    }

    q0 = state->q[0]; q1 = state->q[1]; q2 = state->q[2]; q3 = state->q[3];
    state->euler[0] = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    state->euler[1] = (fabsf(sinp) >= 1.0f) ? copysignf(1.5707963f, sinp) : asinf(sinp);
    state->euler[2] = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    float prev_z = state->pos[2];
    state->pos[2] = (0.92f * (state->pos[2] + state->vel[2] * dt)) + (0.08f * sens->baro_alt);
    state->vel[2] = (state->pos[2] - prev_z) / dt;

    if (sens->gps_fix) {
        state->pos[0] = (0.85f * state->pos[0]) + (0.15f * sens->gps_pos_xy[0]);
        state->pos[1] = (0.85f * state->pos[1]) + (0.15f * sens->gps_pos_xy[1]);
        state->vel[0] = (0.85f * state->vel[0]) + (0.15f * sens->gps_vel_xy[0]);
        state->vel[1] = (0.85f * state->vel[1]) + (0.15f * sens->gps_vel_xy[1]);
    }
}