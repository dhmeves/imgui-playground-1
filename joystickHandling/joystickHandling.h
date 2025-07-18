/**
 * @file joystickHanding.h
 * @brief Joystick control interface for motion controller
 *
 * This module provides joystick input processing for controlling
 * 2D motion systems. It supports both velocity and position control
 * modes with safety features like deadzones and exponential scaling.
 */

#ifndef JOYSTICK_HANDLING_H
#define JOYSTICK_HANDLING_H

#include "motionController_2D.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief Joystick control modes
     */
    typedef enum {
        JOY_MODE_VELOCITY,    /**< Joystick controls velocity (recommended) */
        JOY_MODE_POSITION     /**< Joystick controls position offset */
    } joy_control_mode_e;

    /**
     * @brief Joystick configuration parameters
     */
    typedef struct {
        double deadzone;              /**< Deadzone threshold (0.0 to 1.0, typically 0.1) */
        double expo;                  /**< Exponential curve factor (1.0=linear, 3.0=cubic) */
        double max_vel_scale;         /**< Velocity scaling (0.0 to 1.0) */
        double position_scale;        /**< Position mode: max offset from home */
        joy_control_mode_e mode;      /**< Control mode */
        bool enable_workspace_limits; /**< Enable workspace boundary checking */
        mc2D_vec2_t workspace_min;      /**< Minimum workspace coordinates */
        mc2D_vec2_t workspace_max;      /**< Maximum workspace coordinates */
    } joy_config_ts;

    /**
     * @brief Raw joystick input
     */
    typedef struct {
        double x;     /**< X-axis input (-1.0 to 1.0) */
        double y;     /**< Y-axis input (-1.0 to 1.0) */
        bool enable;  /**< Enable/disable flag (e.g., dead-man switch) */
    } joy_input_ts;

    /**
     * @brief Joystick controller state
     */
    typedef struct {
        mc2D_vec2_t home_position;      /**< Home/center position */
        mc2D_vec2_t current_target;     /**< Current target position */
        mc2D_vec2_t target_velocity;    /**< Target velocity (for feedforward) */
        mc2D_vec2_t actual_position;    /**< Actual system position (for leash mode) */
        joy_config_ts config;          /**< Configuration parameters */
        bool is_active;               /**< Controller active state */
        double time_since_enable;     /**< Time since last enabled (for ramping) */
    } joystk_handlr_ts;

    /**
     * @brief Initialize joystick controller with defaults
     * @param joy Pointer to joystick controller to initialize
     * @param home_pos Home position (center point for position mode)
     */
    void joy_init(joystk_handlr_ts* joy, mc2D_vec2_t home_pos);

    /**
     * @brief Initialize configuration with safe defaults
     * @param config Pointer to configuration structure
     */
    void joy_init_config_default(joy_config_ts* config);

    /**
     * @brief Set joystick control mode
     * @param joy Joystick controller
     * @param mode Control mode (velocity or position)
     */
    void joy_set_mode(joystk_handlr_ts* joy, joy_control_mode_e mode);

    /**
     * @brief Set workspace limits
     * @param joy Joystick controller
     * @param min Minimum coordinates
     * @param max Maximum coordinates
     */
    void joy_set_workspace_limits(joystk_handlr_ts* joy, mc2D_vec2_t min, mc2D_vec2_t max);

    /**
     * @brief Process joystick input and update target
     * @param joy Joystick controller state
     * @param input Raw joystick input
     * @param constraints Motion constraints (for velocity limits)
     * @param dt Time step (seconds)
     */
    void joy_update(joystk_handlr_ts* joy,
        joy_input_ts input,
        mc2D_vec2_t actual_pos,
        mc2D_vec2_t actual_vel,
        const mc2D_constraints_t* constraints,
        double dt);

    /**
     * @brief Get current target position
     * @param joy Joystick controller
     * @return Target position
     */
    mc2D_vec2_t joy_get_target_position(const joystk_handlr_ts* joy);

    /**
     * @brief Get current target velocity
     * @param joy Joystick controller
     * @return Target velocity (for feedforward)
     */
    mc2D_vec2_t joy_get_target_velocity(const joystk_handlr_ts* joy);

    /**
     * @brief Reset controller to home position
     * @param joy Joystick controller
     */
    void joy_reset_to_home(joystk_handlr_ts* joy);

    /**
     * @brief Synchronize target to current position (for smooth transitions)
     * @param joy Joystick controller
     * @param current_pos Current system position
     */
    void joy_sync_to_position(joystk_handlr_ts* joy, mc2D_vec2_t current_pos);

    /**
     * @brief Check if joystick is actively controlling
     * @param joy Joystick controller
     * @return true if active
     */
    bool joy_is_active(const joystk_handlr_ts* joy);

    /**
     * @brief Apply joystick deadzone
     * @param value Input value (-1.0 to 1.0)
     * @param deadzone Deadzone threshold
     * @return Processed value with deadzone applied
     */
    double joy_apply_deadzone(double value, double deadzone);

    /**
     * @brief Apply exponential curve to input
     * @param value Input value (-1.0 to 1.0)
     * @param expo Exponential factor
     * @return Processed value with curve applied
     */
    double joy_apply_expo(double value, double expo);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_HANDLING_H */
