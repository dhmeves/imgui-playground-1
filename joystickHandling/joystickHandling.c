/**
 * @file joystickHandling.c
 * @brief Implementation of joystick control interface
 */

#include "joystickHandling.h"
#include <math.h>

 /* Private helper functions */
static mc2D_vec2_t apply_workspace_limits(mc2D_vec2_t position, const joy_config_ts* config);
static double clamp(double value, double min, double max);
static double smooth_ramp(double time, double ramp_duration);

/* Public API Implementation */

void joy_init(joystk_handlr_ts* joy, mc2D_vec2_t home_pos) {
    joy->home_position = home_pos;
    joy->current_target = home_pos;
    joy->actual_position = home_pos;
    joy->target_velocity.x = 0.0;
    joy->target_velocity.y = 0.0;
    joy->is_active = false;
    joy->time_since_enable = 0.0;

    /* Initialize with safe defaults */
    joy_init_config_default(&joy->config);
}

void joy_init_config_default(joy_config_ts* config) {
    config->deadzone = 0.1;              /* 10% deadzone */
    config->expo = 2.0;                  /* Quadratic response */
    config->max_vel_scale = 0.3;         /* 30% of max velocity for safety */
    config->position_scale = 10.0;       /* 10 unit radius for position mode */
    config->mode = JOY_MODE_VELOCITY;    /* Velocity mode by default */
    config->enable_workspace_limits = false;
    config->workspace_min.x = -100.0;
    config->workspace_min.y = -100.0;
    config->workspace_max.x = 100.0;
    config->workspace_max.y = 100.0;
}

void joy_set_mode(joystk_handlr_ts* joy, joy_control_mode_e mode) {
    /* If switching modes, reset to safe state */
    if (joy->config.mode != mode) {
        joy->config.mode = mode;
        joy->target_velocity.x = 0.0;
        joy->target_velocity.y = 0.0;
        /* In position mode, target should be current position */
        if (mode == JOY_MODE_POSITION) {
            joy->current_target = joy->home_position;
        }
    }
}

void joy_set_workspace_limits(joystk_handlr_ts* joy, mc2D_vec2_t min, mc2D_vec2_t max) {
    joy->config.workspace_min = min;
    joy->config.workspace_max = max;
    joy->config.enable_workspace_limits = true;
}

void joy_update(joystk_handlr_ts* joy,
    joy_input_ts input,
    mc2D_vec2_t actual_pos,
    mc2D_vec2_t actual_vel,
    const mc2D_constraints_t* constraints,
    double dt) {

    joy->actual_position = actual_pos;

    /* Handle enable/disable transitions */
    if (!input.enable) {
        if (joy->is_active) {
            /* Just disabled - stop motion */
            joy->target_velocity.x = 0.0;
            joy->target_velocity.y = 0.0;
            joy->is_active = false;
            joy->time_since_enable = 0.0;
        }
        return;
    }

    /* Track enable time for smooth ramping */
    if (!joy->is_active) {
        joy->is_active = true;
        joy->time_since_enable = 0.0;
    }
    else {
        joy->time_since_enable += dt;
    }

    /* Apply deadzone to inputs */
    double x_processed = joy_apply_deadzone(input.x, joy->config.deadzone);
    double y_processed = joy_apply_deadzone(input.y, joy->config.deadzone);

    /* Apply exponential curve */
    x_processed = joy_apply_expo(x_processed, joy->config.expo);
    y_processed = joy_apply_expo(y_processed, joy->config.expo);

    /* Apply smooth enable ramping (prevents jerky starts) */
    double ramp_factor = smooth_ramp(joy->time_since_enable, 0.5); /* 0.5 second ramp */
    x_processed *= ramp_factor;
    y_processed *= ramp_factor;

    /* Process based on control mode */
    if (joy->config.mode == JOY_MODE_VELOCITY) {
        /* Velocity Control Mode */

        /* Map joystick to desired velocity */
        mc2D_vec2_t commanded_vel;
        commanded_vel.x = x_processed * constraints->max_vel * joy->config.max_vel_scale;
        commanded_vel.y = y_processed * constraints->max_vel * joy->config.max_vel_scale;

        /* Calculate commanded speed */
        double commanded_speed = sqrt(commanded_vel.x * commanded_vel.x +
            commanded_vel.y * commanded_vel.y);

        if (commanded_speed > 0.01) {
            /* Joystick active - integrate velocity to update position */
            joy->current_target.x += commanded_vel.x * dt;
            joy->current_target.y += commanded_vel.y * dt;

            /* Update target velocity for feedforward */
            joy->target_velocity = commanded_vel;

            /* Limit how far ahead target can get */
            double dx = joy->current_target.x - actual_pos.x;
            double dy = joy->current_target.y - actual_pos.y;
            double distance = sqrt(dx * dx + dy * dy);
            double max_distance = 20.0; /* Maximum target lead distance */

            if (distance > max_distance) {
                /* Target too far ahead - constrain it */
                joy->current_target.x = actual_pos.x + (dx / distance) * max_distance;
                joy->current_target.y = actual_pos.y + (dy / distance) * max_distance;
            }
        }
        else {
            /* Joystick at neutral - calculate proper stopping position */

            /* Only update stopping position when first releasing joystick */
            if (fabs(joy->target_velocity.x) > 0.01 || fabs(joy->target_velocity.y) > 0.01) {
                /* Use ACTUAL velocity for accurate stopping distance */
                double speed = sqrt(actual_vel.x * actual_vel.x + actual_vel.y * actual_vel.y);

                if (speed > 0.1) {
                    /* Calculate exact stopping distance */
                    double stop_distance = (speed * speed) / (2.0 * constraints->max_accel);

                    /* Place target at stopping position */
                    joy->current_target.x = actual_pos.x + (actual_vel.x / speed) * stop_distance;
                    joy->current_target.y = actual_pos.y + (actual_vel.y / speed) * stop_distance;
                }
                else {
                    joy->current_target = actual_pos;
                }
            }

            /* Clear commanded velocity */
            joy->target_velocity.x = 0.0;
            joy->target_velocity.y = 0.0;
        }
    }
    else {
        /* Joystick at neutral - IMMEDIATE DECELERATION */

        /* Check if we just transitioned to neutral (velocity was non-zero last frame) */
        if (fabs(joy->target_velocity.x) > 0.01 || fabs(joy->target_velocity.y) > 0.01) {
            /* Just released joystick - calculate stop position ONCE */
            double vel_x = joy->target_velocity.x;
            double vel_y = joy->target_velocity.y;
            double current_speed = sqrt(vel_x * vel_x + vel_y * vel_y);

            /* Calculate stopping distance: d = v²/(2a) */
            double stop_distance = (current_speed * current_speed) / (2.0 * constraints->max_accel);
            stop_distance = stop_distance * 0.7 + 1.0;

            /* Set target at stopping position */
            double vx_norm = vel_x / current_speed;
            double vy_norm = vel_y / current_speed;

            joy->current_target.x = actual_pos.x + vx_norm * stop_distance;
            joy->current_target.y = actual_pos.y + vy_norm * stop_distance;
        }
        /* else: target position remains where it was set - no drift! */

        /* Clear target velocity */
        joy->target_velocity.x = 0.0;
        joy->target_velocity.y = 0.0;
    }

    /* Apply workspace limits if enabled */
    if (joy->config.enable_workspace_limits) {
        mc2D_vec2_t limited = apply_workspace_limits(joy->current_target, &joy->config);

        /* If we hit a limit, zero out velocity in that direction */
        if (limited.x != joy->current_target.x) {
            joy->target_velocity.x = 0.0;
        }
        if (limited.y != joy->current_target.y) {
            joy->target_velocity.y = 0.0;
        }

        joy->current_target = limited;
    }
}

mc2D_vec2_t joy_get_target_position(const joystk_handlr_ts* joy) {
    return joy->current_target;
}

mc2D_vec2_t joy_get_target_velocity(const joystk_handlr_ts* joy) {
    return joy->target_velocity;
}

void joy_reset_to_home(joystk_handlr_ts* joy) {
    joy->current_target = joy->home_position;
    joy->target_velocity.x = 0.0;
    joy->target_velocity.y = 0.0;
    joy->time_since_enable = 0.0;
}

void joy_sync_to_position(joystk_handlr_ts* joy, mc2D_vec2_t current_pos) {
    joy->current_target = current_pos;
    joy->actual_position = current_pos;
    joy->target_velocity.x = 0.0;
    joy->target_velocity.y = 0.0;
    /* Don't reset time_since_enable to maintain smooth control */
}

bool joy_is_active(const joystk_handlr_ts* joy) {
    return joy->is_active;
}

double joy_apply_deadzone(double value, double deadzone) {
    if (fabs(value) < deadzone) {
        return 0.0;
    }

    /* Scale the remaining range to maintain full scale output */
    double sign = (value > 0) ? 1.0 : -1.0;
    return sign * (fabs(value) - deadzone) / (1.0 - deadzone);
}

double joy_apply_expo(double value, double expo) {
    /* Preserve sign while applying exponential curve */
    double sign = (value >= 0) ? 1.0 : -1.0;
    return sign * pow(fabs(value), expo);
}

/* Private helper functions */

static mc2D_vec2_t apply_workspace_limits(mc2D_vec2_t position, const joy_config_ts* config) {
    mc2D_vec2_t limited;
    limited.x = clamp(position.x, config->workspace_min.x, config->workspace_max.x);
    limited.y = clamp(position.y, config->workspace_min.y, config->workspace_max.y);
    return limited;
}

static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static double smooth_ramp(double time, double ramp_duration) {
    if (time >= ramp_duration) {
        return 1.0;
    }
    /* Smooth S-curve ramp */
    double t = time / ramp_duration;
    return t * t * (3.0 - 2.0 * t);
}
