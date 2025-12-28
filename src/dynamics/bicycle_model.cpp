#include "dynamics/bicycle_model.hpp"
#include <stdexcept>
#include <cmath>

namespace gnc {

BicycleModel::BicycleModel(double wheelbase, double dt)
    : wheelbase_(wheelbase), dt_(dt), state_() {
    if (wheelbase <= 0.0) {
        throw std::invalid_argument(
            "BicycleModel: wheelbase must be positive");
    }
    if (dt <= 0.0) {
        throw std::invalid_argument(
            "BicycleModel: timestep must be positive");
    }
}

void BicycleModel::reset(const VehicleState& initial_state) {
    state_ = initial_state;
}

VehicleState BicycleModel::step(const VehicleState& current_state,
                                 const ControlInputs& inputs) {
    /*
     * ========================================================================
     * Runge-Kutta 4th Order (RK4) Integration
     * ========================================================================
     *
     * For a differential equation: dy/dt = f(y, t)
     * The RK4 method approximates y(t + dt) as:
     *
     *   k1 = f(y_n, t_n)
     *   k2 = f(y_n + k1*dt/2, t_n + dt/2)
     *   k3 = f(y_n + k2*dt/2, t_n + dt/2)
     *   k4 = f(y_n + k3*dt, t_n + dt)
     *
     *   y_{n+1} = y_n + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
     *
     * where k1, k2, k3, k4 are slope estimates at different points.
     *
     * ## Why Four Evaluations?
     *
     * - k1: Slope at beginning of interval (like Euler)
     * - k2: Slope at midpoint using k1 (improved estimate)
     * - k3: Slope at midpoint using k2 (further refinement)
     * - k4: Slope at end using k3 (captures end behavior)
     *
     * The weighted average (1-2-2-1 weighting) gives 4th-order accuracy,
     * meaning the local truncation error is O(dt^5) and global error is O(dt^4).
     *
     * ## Comparison with Euler Method
     *
     * **Euler:** y_{n+1} = y_n + dt * f(y_n)
     *   - Only uses slope at start (k1)
     *   - Global error: O(dt) - linear in timestep
     *   - For circular motion, causes spiraling outward (energy drift)
     *
     * **RK4:** Uses weighted average of 4 slopes
     *   - Global error: O(dt^4) - quartic in timestep
     *   - For dt=0.01s: RK4 error ≈ 0.01^4 = 0.00000001 vs Euler error ≈ 0.01
     *   - That's 1,000,000x improvement!
     *
     * ## Educational Exercise
     *
     * To see the difference, replace the RK4 implementation below with Euler:
     *   VehicleState k1 = computeDerivative(current_state, inputs);
     *   return addScaledState(current_state, k1, dt_);
     *
     * Run a circle trajectory for 60 seconds and observe:
     *   - Euler: Vehicle spirals outward (position drift)
     *   - RK4: Vehicle returns to start (stable)
     *
     * ========================================================================
     */

    // Validate inputs to catch programming errors early
    if (!std::isfinite(inputs.acceleration) ||
        !std::isfinite(inputs.steering_angle)) {
        throw std::invalid_argument(
            "BicycleModel::step: Control inputs contain NaN or Inf");
    }

    // RK4 Step 1: Evaluate derivative at current state
    // k1 = f(y_n, t_n)
    VehicleState k1 = computeDerivative(current_state, inputs);

    // RK4 Step 2: Evaluate derivative at midpoint using k1
    // k2 = f(y_n + k1*dt/2, t_n + dt/2)
    VehicleState state_k2 = addScaledState(current_state, k1, dt_ * 0.5);
    VehicleState k2 = computeDerivative(state_k2, inputs);

    // RK4 Step 3: Evaluate derivative at midpoint using k2
    // k3 = f(y_n + k2*dt/2, t_n + dt/2)
    VehicleState state_k3 = addScaledState(current_state, k2, dt_ * 0.5);
    VehicleState k3 = computeDerivative(state_k3, inputs);

    // RK4 Step 4: Evaluate derivative at endpoint using k3
    // k4 = f(y_n + k3*dt, t_n + dt)
    VehicleState state_k4 = addScaledState(current_state, k3, dt_);
    VehicleState k4 = computeDerivative(state_k4, inputs);

    // Combine slopes with RK4 weighting: (1*k1 + 2*k2 + 2*k3 + 1*k4) / 6
    // This weighted average minimizes truncation error
    VehicleState weighted_slope;
    weighted_slope.x = (k1.x + 2.0*k2.x + 2.0*k3.x + k4.x) / 6.0;
    weighted_slope.y = (k1.y + 2.0*k2.y + 2.0*k3.y + k4.y) / 6.0;
    weighted_slope.theta = (k1.theta + 2.0*k2.theta + 2.0*k3.theta + k4.theta) / 6.0;
    weighted_slope.v = (k1.v + 2.0*k2.v + 2.0*k3.v + k4.v) / 6.0;

    // Final update: y_{n+1} = y_n + dt * weighted_slope
    VehicleState new_state = addScaledState(current_state, weighted_slope, dt_);

    // Update timestamp
    new_state.timestamp = current_state.timestamp + dt_;

    // Sanity check: Ensure numerical stability (catch bugs early)
    if (!std::isfinite(new_state.x) || !std::isfinite(new_state.y) ||
        !std::isfinite(new_state.theta) || !std::isfinite(new_state.v)) {
        throw std::runtime_error(
            "BicycleModel::step: Numerical instability detected (non-finite state). "
            "This may indicate excessively large inputs or timestep.");
    }

    // Update internal state
    state_ = new_state;

    return new_state;
}

VehicleState BicycleModel::computeDerivative(const VehicleState& state,
                                              const ControlInputs& inputs) const {
    /*
     * ========================================================================
     * Bicycle Model Kinematic Equations
     * ========================================================================
     *
     * These equations describe how the vehicle state changes over time
     * based on its current configuration and control inputs.
     *
     * ## Derivation from First Principles
     *
     * Consider a bicycle with:
     *   - Rear wheel at position (x_r, y_r)
     *   - Front wheel at distance L ahead
     *   - Vehicle heading θ (angle from x-axis)
     *   - Front wheel steered at angle δ relative to vehicle body
     *
     * ### Position Dynamics
     *
     * The rear wheel velocity is always aligned with vehicle heading:
     *   v_rear = v * [cos(θ), sin(θ)]
     *
     * Therefore:
     *   dx/dt = v * cos(θ)    [x-component of velocity]
     *   dy/dt = v * sin(θ)    [y-component of velocity]
     *
     * ### Heading Dynamics
     *
     * The instantaneous center of rotation (ICR) is found by extending
     * perpendiculars from both wheels. Basic geometry gives:
     *
     *   Turn radius: R = L / tan(δ)
     *   Angular velocity: ω = v / R = v * tan(δ) / L
     *
     * Therefore:
     *   dθ/dt = (v / L) * tan(δ)
     *
     * ### Velocity Dynamics
     *
     * Assuming point mass (no rotational inertia), acceleration directly
     * changes forward velocity:
     *   dv/dt = a
     *
     * ## Physical Limits (Not Enforced Here, But Consider in Practice)
     *
     * - Steering angle: |δ| < 0.6 rad (≈35°) for typical vehicles
     * - Acceleration: -8 m/s² < a < 5 m/s² (braking vs acceleration)
     * - Velocity: v ≥ 0 (we allow reverse, but kinematic model less accurate)
     *
     * ========================================================================
     */

    VehicleState derivative;

    // dx/dt = v * cos(θ)
    // Longitudinal component of velocity in global frame
    derivative.x = state.v * std::cos(state.theta);

    // dy/dt = v * sin(θ)
    // Lateral component of velocity in global frame
    derivative.y = state.v * std::sin(state.theta);

    // dθ/dt = (v / L) * tan(δ)
    // Yaw rate (angular velocity about vertical axis)
    // Note: tan(δ) can be large for sharp turns, but steering is typically limited
    derivative.theta = (state.v / wheelbase_) * std::tan(inputs.steering_angle);

    // dv/dt = a
    // Linear acceleration (change in forward speed)
    derivative.v = inputs.acceleration;

    // Timestamp derivative is always 1 (dt/dt = 1)
    derivative.timestamp = 1.0;

    return derivative;
}

VehicleState BicycleModel::addScaledState(const VehicleState& state1,
                                           const VehicleState& state2,
                                           double scale) const {
    /*
     * Helper function for RK4 intermediate calculations.
     * Computes: state1 + scale * state2
     *
     * Used for:
     *   - y_n + k*dt/2 (midpoint evaluations)
     *   - y_n + k*dt (endpoint evaluation)
     *   - y_n + dt*weighted_slope (final update)
     */
    VehicleState result;
    result.x = state1.x + scale * state2.x;
    result.y = state1.y + scale * state2.y;
    result.theta = state1.theta + scale * state2.theta;
    result.v = state1.v + scale * state2.v;
    result.timestamp = state1.timestamp + scale * state2.timestamp;
    return result;
}

}  // namespace gnc
