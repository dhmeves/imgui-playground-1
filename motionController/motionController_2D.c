/**
 * @file motionController_2D.c
 * @brief Implementation of 2D motion controller
 */

#include "motionController_2D.h"
#include <math.h>

 /* Helper internal function prototypes */
static double vec2_magnitude(mc2D_vec2_t v);
static mc2D_vec2_t vec2_normalize(mc2D_vec2_t v);
static double calculate_stopping_distance(double current_speed, double max_accel);
static bool is_at_target(mc2D_vec2_t pos, mc2D_vec2_t target_pos, double tolerance);
static bool is_stopped(mc2D_vec2_t vel, double tolerance);

/* Public API Implementation */

void mc2D_init(mc2D_state_t* state, mc2D_vec2_t initial_pos, mc2D_vec2_t initial_vel) {
    state->pos = initial_pos;
    state->vel = initial_vel;
    state->state = MC_STATE_ACCELERATING;
    state->settled_count = 0;
}

void mc2D_init_constraints_default(mc2D_constraints_t* constraints) {
    constraints->max_vel = 10.0;
    constraints->max_accel = 5.0;
    constraints->position_tol = 0.01;
    constraints->velocity_tol = 0.05;
    constraints->homing_distance = 0.5;
    constraints->homing_gain = 2.0;
    constraints->settle_cycles = 5;
}

mc2D_vec2_t mc2D_compute_control(mc2D_state_t* state,
    mc2D_vec2_t target_pos,
    mc2D_vec2_t target_vel,
    const mc2D_constraints_t* limits,
    double dt) {

    /* Compute position error */
    mc2D_vec2_t error = {
        target_pos.x - state->pos.x,
        target_pos.y - state->pos.y
    };

    double error_mag = vec2_magnitude(error);
    double current_speed = vec2_magnitude(state->vel);

    /* State machine logic */
    bool at_target = is_at_target(state->pos, target_pos, limits->position_tol);
    bool stopped = is_stopped(state->vel, limits->velocity_tol);

    /* Check for settled state */
    if (at_target && stopped) {
        state->settled_count++;
        if (state->settled_count > limits->settle_cycles) {
            state->state = MC_STATE_SETTLED;
        }
    }
    else {
        state->settled_count = 0;

        /* Determine motion state */
        if (error_mag < limits->homing_distance) {
            state->state = MC_STATE_HOMING;
        }
        else {
            double stop_dist = calculate_stopping_distance(current_speed, limits->max_accel);
            if (error_mag <= stop_dist * 1.2) {
                state->state = MC_STATE_DECELERATING;
            }
            else if (current_speed >= limits->max_vel * 0.95) {
                state->state = MC_STATE_CRUISING;
            }
            else {
                state->state = MC_STATE_ACCELERATING;
            }
        }
    }

    /* Compute desired velocity based on state */
    mc2D_vec2_t desired_vel = { 0, 0 };

    switch (state->state) {
    case MC_STATE_SETTLED:
        /* At target and settled - output zero */
        desired_vel.x = 0;
        desired_vel.y = 0;
        break;

    case MC_STATE_HOMING:
        /* Low-gain proportional control for final approach */
        desired_vel.x = error.x * limits->homing_gain;
        desired_vel.y = error.y * limits->homing_gain;

        /* Add small feedforward from target velocity */
        desired_vel.x += target_vel.x * 0.1;
        desired_vel.y += target_vel.y * 0.1;
        break;

    case MC_STATE_DECELERATING: {
        /* Smooth deceleration to stop at target */
        mc2D_vec2_t direction = vec2_normalize(error);
        double desired_speed = sqrt(2.0 * limits->max_accel * error_mag * 0.9);

        if (desired_speed > limits->max_vel) {
            desired_speed = limits->max_vel;
        }

        desired_vel.x = direction.x * desired_speed;
        desired_vel.y = direction.y * desired_speed;
        break;
    }

    case MC_STATE_CRUISING:
    case MC_STATE_ACCELERATING: {
        /* Full speed toward target */
        mc2D_vec2_t direction = vec2_normalize(error);
        double desired_speed = limits->max_vel;

        desired_vel.x = direction.x * desired_speed;
        desired_vel.y = direction.y * desired_speed;

        /* Add target velocity feedforward */
        desired_vel.x += target_vel.x * 0.8;
        desired_vel.y += target_vel.y * 0.8;
        break;
    }
    }

    /* Enforce velocity limits */
    double vel_mag = vec2_magnitude(desired_vel);
    if (vel_mag > limits->max_vel) {
        desired_vel.x = (desired_vel.x / vel_mag) * limits->max_vel;
        desired_vel.y = (desired_vel.y / vel_mag) * limits->max_vel;
    }

    /* Compute required acceleration */
    mc2D_vec2_t accel_cmd = {
        (desired_vel.x - state->vel.x) / dt,
        (desired_vel.y - state->vel.y) / dt
    };

    /* Apply acceleration limits (gentler in homing mode) */
    double accel_limit = (state->state == MC_STATE_HOMING) ?
        limits->max_accel * 0.9 : limits->max_accel;

    double accel_mag = vec2_magnitude(accel_cmd);
    if (accel_mag > accel_limit) {
        accel_cmd.x = (accel_cmd.x / accel_mag) * accel_limit;
        accel_cmd.y = (accel_cmd.y / accel_mag) * accel_limit;
    }

    return accel_cmd;
}

void mc2D_update_state(mc2D_state_t* state, mc2D_vec2_t accel_cmd, double dt) {
    /* Update velocity */
    state->vel.x += accel_cmd.x * dt;
    state->vel.y += accel_cmd.y * dt;

    /* Update position */
    state->pos.x += state->vel.x * dt;
    state->pos.y += state->vel.y * dt;
}

const char* mc2D_get_state_name(mc2D_motion_state_t state) {
    switch (state) {
    case MC_STATE_ACCELERATING: return "ACCELERATING";
    case MC_STATE_CRUISING:     return "CRUISING";
    case MC_STATE_DECELERATING: return "DECELERATING";
    case MC_STATE_HOMING:       return "HOMING";
    case MC_STATE_SETTLED:      return "SETTLED";
    default:                    return "UNKNOWN";
    }
}

bool mc2D_is_settled(const mc2D_state_t* state) {
    return state->state == MC_STATE_SETTLED;
}

double mc2D_get_distance_to_target(const mc2D_state_t* state, mc2D_vec2_t target_pos) {
    double dx = target_pos.x - state->pos.x;
    double dy = target_pos.y - state->pos.y;
    return sqrt(dx * dx + dy * dy);
}

double mc2D_get_speed(const mc2D_state_t* state) {
    return vec2_magnitude(state->vel);
}

/* Private helper functions */

static double vec2_magnitude(mc2D_vec2_t v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

static mc2D_vec2_t vec2_normalize(mc2D_vec2_t v) {
    double mag = vec2_magnitude(v);
    if (mag > 0.0001) {
        mc2D_vec2_t result = { v.x / mag, v.y / mag };
        return result;
    }
    mc2D_vec2_t zero = { 0, 0 };
    return zero;
}

static double calculate_stopping_distance(double current_speed, double max_accel) {
    return (current_speed * current_speed) / (2.0 * max_accel);
}

static bool is_at_target(mc2D_vec2_t pos, mc2D_vec2_t target_pos, double tolerance) {
    double dx = target_pos.x - pos.x;
    double dy = target_pos.y - pos.y;
    return (dx * dx + dy * dy) <= (tolerance * tolerance);
}

static bool is_stopped(mc2D_vec2_t vel, double tolerance) {
    return (vel.x * vel.x + vel.y * vel.y) <= (tolerance * tolerance);
}
