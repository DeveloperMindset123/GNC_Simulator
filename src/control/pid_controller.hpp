#ifndef GNC_SIMULATOR_PID_CONTROLLER_HPP
#define GNC_SIMULATOR_PID_CONTROLLER_HPP

// above header guard prevents double include problem

namespace gnc
{

    /**
     * @brief PID (Proportional-Integral-Derivative) Controller with Anti-Windup
     *
     * ## Control Theory Background
     *
     * A PID controller computes a control output based on the error between
     * a desired setpoint and the measured value:
     *
     *   error(t) = setpoint(t) - measurement(t)
     *   output(t) = Kp*error + Ki*∫error*dt + Kd*derror/dt
     *
     * ### The Three Terms
     *
     * **Proportional (P):** Kp * error
     *   - Responds immediately to current error
     *   - Larger Kp → faster response but more overshoot
     *   - Alone, typically leaves steady-state error
     *   - Example: If error = 2 m/s and Kp = 1.5, output = 3 m/s²
     *
     * **Integral (I):** Ki * ∫error*dt
     *   - Accumulates past errors over time
     *   - Eliminates steady-state error (error that P and D can't fix)
     *   - Larger Ki → faster steady-state convergence but more overshoot
     *   - WARNING: Can cause "windup" if output saturates (see below)
     *   - Example: If error stays at 0.1 m/s for 5s and Ki = 0.5,
     *              integral term grows to 0.5 * 0.5 = 0.25 m/s²
     *
     * **Derivative (D):** Kd * derror/dt
     *   - Predicts future error based on rate of change
     *   - Provides damping (reduces overshoot and oscillations)
     *   - Larger Kd → more damping but sensitive to noise
     *   - Example: If error is decreasing at 2 m/s per second and Kd = 0.3,
     *              derivative term = -0.6 m/s² (opposes overshoot)
     *
     * ## The Integral Windup Problem
     *
     * **What is it?**
     * When the output saturates (hits min/max limits), the error persists,
     * causing the integral term to grow unbounded. When setpoint changes or
     * error finally decreases, the huge integral takes a long time to "unwind,"
     * causing large overshoot.
     *
     * **Example Scenario:**
     * - Velocity PID wants 10 m/s but vehicle max accel is 5 m/s²
     * - Output saturates at 5 m/s² for 10 seconds
     * - Integral term accumulates to huge value
     * - When velocity reaches 10 m/s, integral is still large
     * - Vehicle overshoots to 15 m/s before integral unwinds
     *
     * **Our Solution: Integral Clamping**
     * We limit the integral term to [-integral_max, +integral_max].
     * This prevents unbounded growth while still allowing integral action.
     *
     * Alternative methods include:
     * - Conditional integration (stop integrating when saturated)
     * - Back-calculation (reduce integral based on saturation)
     *
     * ## Tuning Guidelines (Ziegler-Nichols Method)
     *
     * 1. **Start with P only** (Ki=0, Kd=0):
     *    - Increase Kp until system oscillates
     *    - Reduce Kp to 60% of that value (Kp_critical)
     *
     * 2. **Add I term**:
     *    - Start with Ki = Kp / 10
     *    - Increase until steady-state error disappears
     *    - If overshooting, reduce Ki
     *
     * 3. **Add D term**:
     *    - Start with Kd = Kp / 20
     *    - Increase to reduce overshoot/oscillations
     *    - If noisy or unstable, reduce Kd
     *
     * ## Manual Tuning Tips
     *
     * - Kp too high → oscillations, instability
     * - Kp too low → slow response, large steady-state error
     * - Ki too high → overshoot, oscillations
     * - Ki too low → persistent steady-state error
     * - Kd too high → noise amplification, high-frequency oscillations
     * - Kd too low → excessive overshoot
     *
     * ## Use Cases in GNC Simulator
     *
     * **Velocity Control:**
     * - Error: v_desired - v_actual
     * - Output: acceleration command [m/s²]
     * - Typical gains: Kp=2.0, Ki=0.5, Kd=0.1
     *
     * **Steering Control:**
     * - Error: theta_desired - theta_actual (normalized to [-π, π])
     * - Output: steering angle [rad]
     * - Typical gains: Kp=1.5, Ki=0.0 (often set to zero!), Kd=0.3
     * - Note: Ki=0 for steering avoids windup during sharp turns
     */
    class PIDController
    {
    public:
        /**
         * @brief Construct a PID controller with anti-windup
         *
         * @param kp Proportional gain (≥ 0)
         * @param ki Integral gain (≥ 0)
         * @param kd Derivative gain (≥ 0)
         * @param output_min Minimum output value (saturation limit)
         * @param output_max Maximum output value (saturation limit)
         * @param integral_max Maximum integral term magnitude (anti-windup)
         *
         * @throws std::invalid_argument if output_min >= output_max
         * @throws std::invalid_argument if integral_max <= 0
         */
        PIDController(double kp, double ki, double kd,
                      double output_min, double output_max,
                      double integral_max = 10.0);

        /**
         * @brief Compute control output for current error
         *
         * This should be called at regular intervals (constant dt) for best performance.
         *
         * @param error Current error (setpoint - measurement)
         * @param dt Time since last call [s]
         *           Must be > 0 and approximately constant for accurate integral/derivative
         * @return Saturated control output in [output_min, output_max]
         */
        double compute(double error, double dt);

        /**
         * @brief Reset controller state
         *
         * Clears integral accumulation and derivative history.
         * Call this when starting a new trajectory or after large disturbances.
         */
        void reset();

        // Parameter modification (for online tuning)

        /**
         * @brief Update PID gains during runtime
         *
         * Useful for adaptive control or manual tuning experiments.
         * Does NOT reset integral or derivative state.
         *
         * @param kp New proportional gain
         * @param ki New integral gain
         * @param kd New derivative gain
         */
        void setGains(double kp, double ki, double kd);

        // Diagnostic getters (useful for logging and tuning)

        /**
         * @brief Get proportional term from last compute() call
         * @return Kp * error
         */
        double getProportionalTerm() const { return p_term_; }

        /**
         * @brief Get integral term from last compute() call
         * @return Ki * ∫error*dt (clamped to [-integral_max, +integral_max])
         */
        double getIntegralTerm() const { return i_term_; }

        /**
         * @brief Get derivative term from last compute() call
         * @return Kd * derror/dt
         */
        double getDerivativeTerm() const { return d_term_; }

        /**
         * @brief Get raw accumulated integral (before Ki multiplication)
         * @return ∫error*dt (clamped)
         */
        double getIntegralAccumulation() const { return integral_; }

    private:
        // PID gains
        double kp_; // Proportional gain
        double ki_; // Integral gain
        double kd_; // Derivative gain

        // Output saturation limits
        double output_min_;
        double output_max_;

        // Anti-windup parameter
        double integral_max_; // Maximum |integral| value
        double integral_;     // Accumulated integral of error

        // Derivative calculation
        double prev_error_; // Error from previous call (for derivative)
        bool first_run_;    // Flag to handle first compute() call

        // Diagnostic storage (last computed values)
        double p_term_;
        double i_term_;
        double d_term_;
    };

} // namespace gnc

#endif // GNC_SIMULATOR_PID_CONTROLLER_HPP
