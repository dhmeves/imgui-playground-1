/**
 * @file motion_controller.h
 * @brief 2D Motion Controller with velocity and acceleration limits
 *
 * This controller provides smooth motion control for tracking moving targets
 * while respecting velocity and acceleration constraints. It includes
 * anti-oscillation features through state management and position tolerances.
 */

#ifndef MOTION_CONTROLLER_2D_H
#define MOTION_CONTROLLER_2D_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
    * @brief 2D vector structure
    */
typedef struct {
    double x;
    double y;
} mc2D_vec2_t;

/**
    * @brief Motion constraints configuration
    */
typedef struct {
    double max_vel;          /**< Maximum velocity magnitude (units/sec) */
    double max_accel;        /**< Maximum acceleration magnitude (units/sec^2) */
    double position_tol;     /**< Position tolerance for "at target" (units) */
    double velocity_tol;     /**< Velocity tolerance for "settled" (units/sec) */
    double homing_distance;  /**< Distance at which to switch to homing mode (units) */
    double homing_gain;      /**< Proportional gain for final homing (typically 1.0-5.0) */
    int settle_cycles;       /**< Number of cycles to confirm settled state */
} mc2D_constraints_t;

/**
    * @brief Motion controller states
    */
typedef enum {
    MC_STATE_ACCELERATING,   /**< Accelerating toward target */
    MC_STATE_CRUISING,       /**< At maximum velocity */
    MC_STATE_DECELERATING,   /**< Decelerating to stop at target */
    MC_STATE_HOMING,         /**< Final approach with reduced gain */
    MC_STATE_SETTLED         /**< At target and stopped */
} mc2D_motion_state_t;

/**
    * @brief Motion controller state
    */
typedef struct {
    mc2D_vec2_t pos;              /**< Current position */
    mc2D_vec2_t vel;              /**< Current velocity */
    mc2D_motion_state_t state;    /**< Current motion state */
    int settled_count;          /**< Counter for settled state debouncing */
} mc2D_state_t;

/**
    * @brief Initialize motion controller state
    * @param state Pointer to state structure to initialize
    * @param initial_pos Initial position
    * @param initial_vel Initial velocity
    */
void mc2D_init(mc2D_state_t* state, mc2D_vec2_t initial_pos, mc2D_vec2_t initial_vel);

/**
    * @brief Initialize constraints with default values
    * @param constraints Pointer to constraints structure to initialize
    */
void mc2D_init_constraints_default(mc2D_constraints_t* constraints);

/**
    * @brief Compute acceleration command for current cycle
    * @param state Current state (will be updated with new motion state)
    * @param target_pos Target position
    * @param target_vel Target velocity (for feedforward)
    * @param constraints Motion constraints
    * @param dt Time step (seconds)
    * @return Acceleration command
    */
mc2D_vec2_t mc2D_compute_control(mc2D_state_t* state,
    mc2D_vec2_t target_pos,
    mc2D_vec2_t target_vel,
    const mc2D_constraints_t* constraints,
    double dt);

/**
    * @brief Update state based on acceleration command
    * @param state State to update
    * @param accel_cmd Acceleration command from mc_compute_control
    * @param dt Time step (seconds)
    */
void mc2D_update_state(mc2D_state_t* state, mc2D_vec2_t accel_cmd, double dt);

/**
    * @brief Get human-readable name for motion state
    * @param state Motion state
    * @return String representation of state
    */
const char* mc2D_get_state_name(mc2D_motion_state_t state);

/**
    * @brief Check if controller has reached target and settled
    * @param state Current state
    * @return true if settled at target
    */
bool mc2D_is_settled(const mc2D_state_t* state);

/**
    * @brief Get distance to target
    * @param state Current state
    * @param target_pos Target position
    * @return Euclidean distance to target
    */
double mc2D_get_distance_to_target(const mc2D_state_t* state, mc2D_vec2_t target_pos);

/**
    * @brief Get current speed
    * @param state Current state
    * @return Current speed (velocity magnitude)
    */
double mc2D_get_speed(const mc2D_state_t* state);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H */
