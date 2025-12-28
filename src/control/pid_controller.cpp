#include "control/pid_controller.hpp"
#include <stdexcept>
#include <algorithm>  // For std::clamp
#include <cmath>

namespace gnc {

PIDController::PIDController(double kp, double ki, double kd,
                             double output_min, double output_max,
                             double integral_max)
    : kp_(kp), ki_(ki), kd_(kd),
      output_min_(output_min), output_max_(output_max),
      integral_max_(integral_max),
      integral_(0.0), prev_error_(0.0), first_run_(true),
      p_term_(0.0), i_term_(0.0), d_term_(0.0) {

    // Validate parameters
    if (output_min >= output_max) {
        throw std::invalid_argument(
            "PIDController: output_min must be < output_max");
    }

    if (integral_max <= 0.0) {
        throw std::invalid_argument(
            "PIDController: integral_max must be positive");
    }

    // Allow negative gains for special cases, but warn if unintentional
    // (In practice, all gains should be non-negative for standard PID)
}

void PIDController::reset() {
    /*
     * Reset the controller's internal state.
     *
     * This is important when:
     * 1. Starting a new trajectory (old integral is irrelevant)
     * 2. Large setpoint changes (prevents derivative kick)
     * 3. After disturbances or failures (start fresh)
     *
     * Example: When switching from circle to straight-line trajectory,
     * reset both velocity and steering PIDs to avoid transient errors.
     */
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_run_ = true;
    p_term_ = 0.0;
    i_term_ = 0.0;
    d_term_ = 0.0;
}

void PIDController::setGains(double kp, double ki, double kd) {
    /*
     * Update gains during runtime.
     *
     * Educational Note: This enables online tuning experiments.
     * Try running the simulator and adjusting gains to see effects:
     * - Double Kp → faster response, more oscillation
     * - Zero Ki → steady-state error appears
     * - Zero Kd → more overshoot
     */
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

double PIDController::compute(double error, double dt) {
    /*
     * ========================================================================
     * PID Control Law Implementation
     * ========================================================================
     *
     * The discrete-time PID equation:
     *
     *   output(k) = Kp * e(k) + Ki * sum(e) * dt + Kd * (e(k) - e(k-1)) / dt
     *
     * where:
     *   e(k) = current error
     *   sum(e) = accumulated error over time (integral)
     *   (e(k) - e(k-1)) / dt = rate of error change (derivative)
     *
     * ## Implementation Details
     *
     * 1. **Proportional Term**: Direct multiplication, no history needed
     *
     * 2. **Integral Term**: Trapezoidal rule integration
     *    integral += error * dt
     *    This is simple rectangular integration. For higher accuracy,
     *    could use: integral += (error + prev_error) * dt / 2 (trapezoid)
     *
     * 3. **Derivative Term**: Backward difference
     *    derivative = (error - prev_error) / dt
     *    Note: This is sensitive to noise! In practice, often filtered.
     *    Advanced: Use derivative of measurement (not error) to avoid
     *              "derivative kick" on setpoint changes.
     *
     * ## Anti-Windup Strategy
     *
     * Before using the integral, we clamp it:
     *   integral = clamp(integral, -integral_max, +integral_max)
     *
     * This prevents unbounded growth during saturation.
     * The output is then saturated separately.
     *
     * ========================================================================
     */

    if (dt <= 0.0) {
        throw std::invalid_argument(
            "PIDController::compute: dt must be positive");
    }

    // 1. Proportional term: immediate response to current error
    p_term_ = kp_ * error;

    // 2. Integral term: accumulate error over time
    integral_ += error * dt;

    // Anti-windup: Clamp integral to prevent unbounded growth
    // This is critical when output saturates for extended periods
    integral_ = std::clamp(integral_, -integral_max_, integral_max_);

    i_term_ = ki_ * integral_;

    // 3. Derivative term: predict future error based on trend
    double derivative = 0.0;
    if (!first_run_) {
        // derivative = rate of change of error
        derivative = (error - prev_error_) / dt;
    } else {
        // On first call, we have no previous error
        // Set derivative to 0 to avoid spurious spike
        derivative = 0.0;
        first_run_ = false;
    }

    d_term_ = kd_ * derivative;

    // 4. Combine all three terms
    double output = p_term_ + i_term_ + d_term_;

    // 5. Apply output saturation limits
    // This protects actuators from unrealistic commands
    output = std::clamp(output, output_min_, output_max_);

    // 6. Store current error for next derivative calculation
    prev_error_ = error;

    /*
     * Debugging Tip: Log p_term_, i_term_, d_term_ to understand behavior
     * Example interpretations:
     *
     * - p_term_ dominates → Error is large, system reacting strongly
     * - i_term_ dominates → Long-term steady-state correction active
     * - d_term_ large negative → Error decreasing fast, applying brake
     * - output == output_max_ → Saturated, check for integral windup
     */

    return output;
}

}  // namespace gnc
