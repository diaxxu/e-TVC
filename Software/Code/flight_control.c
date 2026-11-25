#include "flight_control.h"
#include <math.h>

#define DEG2RAD(x) ((x) * 0.017453292519943295f)
#define CLAMP(val, min_v, max_v) ((val) < (min_v) ? (min_v) : ((val) > (max_v) ? (max_v) : (val)))

typedef struct {
    float kp, ki, kd;
    float limit;
    float i_limit;
    float integral;
    float prev_in;
    float deriv_filt;
} PID_Controller_t;

static PID_Controller_t pid_z     = {1.5f,  0.05f, 0.8f,  4.0f,          1.5f, 0, 0, 0};
static PID_Controller_t pid_vz    = {30.0f, 15.0f, 5.0f,  400.0f,        50.0f, 0, 0, 0};
static PID_Controller_t pid_x     = {0.25f, 0.0f,  0.45f, 2.0f,          0.0f, 0, 0, 0};
static PID_Controller_t pid_vx    = {0.20f, 0.0f,  0.40f, DEG2RAD(10.0f),0.0f, 0, 0, 0};
static PID_Controller_t pid_theta = {0.90f, 0.01f, 0.70f, DEG2RAD(15.0f),DEG2RAD(3.75f), 0, 0, 0};
static PID_Controller_t pid_y     = {0.25f, 0.0f,  0.45f, 2.0f,          0.0f, 0, 0, 0};
static PID_Controller_t pid_vy    = {-0.20f,0.0f, -0.40f, DEG2RAD(10.0f),0.0f, 0, 0, 0};
static PID_Controller_t pid_phi   = {0.90f, 0.01f, 0.70f, DEG2RAD(15.0f),DEG2RAD(3.75f), 0, 0, 0};

static FlightState_t current_state = STATE_DISARMED;

static void resetPID(PID_Controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_in = 0.0f;
    pid->deriv_filt = 0.0f;
}

static float computePID(float sp, float in, float dt, PID_Controller_t *pid) {
    float err = sp - in;
    pid->integral = CLAMP(pid->integral + err * dt, -pid->i_limit, pid->i_limit);

    float deriv_raw = -(in - pid->prev_in) / dt;
    pid->deriv_filt = (0.8f * pid->deriv_filt) + (0.2f * deriv_raw);
    pid->prev_in = in;

    float out = (pid->kp * err) + (pid->ki * pid->integral) + (pid->kd * pid->deriv_filt);
    return CLAMP(out, -pid->limit, pid->limit);
}

void FlightControl_Init(void) {
    resetPID(&pid_z); resetPID(&pid_vz);
    resetPID(&pid_x); resetPID(&pid_vx); resetPID(&pid_theta);
    resetPID(&pid_y); resetPID(&pid_vy); resetPID(&pid_phi);
    current_state = STATE_DISARMED;
}

void FlightControl_Update200Hz(float dt, const VehicleState_t *state, const RC_Input_t *rc,
                               uint32_t now_ms, uint16_t *esc_pwm, uint16_t *servo_x_pwm,
                               uint16_t *servo_y_pwm, uint16_t *chute_pwm) {
    static float active_cmd_x = 0.0f;
    static float active_cmd_y = 0.0f;
    static float target_pos[3] = {0.0f, 0.0f, 5.0f};
    const float deadband = DEG2RAD(0.3f);
    const float max_lean = DEG2RAD(10.0f);

    bool armed = CRSF_IsArmed(rc, now_ms);
    bool chute_req = CRSF_IsParachuteTriggered(rc);

    if (fabsf(state->euler[0]) > DEG2RAD(55.0f) || fabsf(state->euler[1]) > DEG2RAD(55.0f)) {
        if (current_state != STATE_DISARMED) chute_req = true;
    }

    if (chute_req) {
        current_state = STATE_PARACHUTE;
        *esc_pwm = 1000;
        *servo_x_pwm = 1500;
        *servo_y_pwm = 1500;
        *chute_pwm = 2000;
        return;
    } else {
        *chute_pwm = 1000;
    }

    if (!armed) {
        current_state = STATE_DISARMED;
        FlightControl_Init();
        *esc_pwm = 1000;
        *servo_x_pwm = 1500;
        *servo_y_pwm = 1500;
        return;
    }

    if (current_state == STATE_DISARMED) {
        current_state = STATE_LAUNCH_HOVER;
        target_pos[0] = 0.0f;
        target_pos[1] = 0.0f;
        target_pos[2] = 5.0f;
    }

    if (CRSF_IsAutoMode(rc)) {
        current_state = STATE_WAYPOINT_NAV;
        target_pos[0] = 10.0f;
        target_pos[1] = 0.0f;
        target_pos[2] = 10.0f;
    } else {
        float stick_x = ((float)rc->ch[1] - 992.0f) / 800.0f;
        float stick_y = ((float)rc->ch[0] - 992.0f) / 800.0f;
        float stick_z = ((float)rc->ch[2] - 172.0f) / 1600.0f;

        if (fabsf(stick_x) > 0.05f) target_pos[0] += stick_x * 2.0f * dt;
        if (fabsf(stick_y) > 0.05f) target_pos[1] += stick_y * 2.0f * dt;
        target_pos[2] = CLAMP(stick_z * 15.0f, 0.5f, 15.0f);
    }

    float target_vz = computePID(target_pos[2], state->pos[2], dt, &pid_z);
    float throttle_offset = computePID(target_vz, state->vel[2], dt, &pid_vz);
    *esc_pwm = (uint16_t)CLAMP(1450.0f + throttle_offset, 1100.0f, 1900.0f);

    float target_vx = computePID(target_pos[0], state->pos[0], dt, &pid_x);
    float target_theta = CLAMP(computePID(target_vx, state->vel[0], dt, &pid_vx), -max_lean, max_lean);

    float target_vy = computePID(target_pos[1], state->pos[1], dt, &pid_y);
    float target_phi = CLAMP(computePID(target_vy, state->vel[1], dt, &pid_vy), -max_lean, max_lean);

    float raw_cmd_x = computePID(target_theta, state->euler[1], dt, &pid_theta);
    float raw_cmd_y = computePID(target_phi, state->euler[0], dt, &pid_phi);

    if (fabsf(raw_cmd_x - active_cmd_x) > deadband) active_cmd_x = raw_cmd_x;
    if (fabsf(raw_cmd_y - active_cmd_y) > deadband) active_cmd_y = raw_cmd_y;

    float scale = 500.0f / DEG2RAD(15.0f);
    *servo_x_pwm = (uint16_t)CLAMP(1500.0f + (active_cmd_x * scale), 1000.0f, 2000.0f);
    *servo_y_pwm = (uint16_t)CLAMP(1500.0f + (active_cmd_y * scale), 1000.0f, 2000.0f);
}