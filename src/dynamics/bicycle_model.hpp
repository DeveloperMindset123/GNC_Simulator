#ifndef GNC_SIMULATOR_BICYCLE_MODEL_HPP
#define GNC_SIMULATOR_BICYCLE_MODEL_HPP

#include <cmath>

namespace gnc {

/**
 * @brief Vehicle state representation in 2D
 *
 * The state vector contains the complete configuration of the vehicle
 * in the global coordinate frame at a given time instant.
 */
struct VehicleState {
    double x;           // Position x in global frame [m]
    double y;           // Position y in global frame [m]
    double theta;       // Heading angle (yaw) from x-axis [rad]
    double v;           // Forward velocity [m/s]
    double timestamp;   // Simulation time [s]

    VehicleState()
        : x(0.0), y(0.0), theta(0.0), v(0.0), timestamp(0.0) {}

    VehicleState(double x_, double y_, double theta_, double v_, double t_)
        : x(x_), y(y_), theta(theta_), v(v_), timestamp(t_) {}
};

/**
 * @brief Control inputs to the vehicle
 *
 * These are the commanded inputs that drive the vehicle dynamics.
 * In a real system, these would be sent to actuators (throttle, steering).
 */
struct ControlInputs {
    double acceleration;     // Linear acceleration command [m/s^2]
    double steering_angle;   // Front wheel steering angle [rad]

    ControlInputs() : acceleration(0.0), steering_angle(0.0) {}

    ControlInputs(double a, double delta)
        : acceleration(a), steering_angle(delta) {}
};

/**
 * @brief Reference trajectory point
 *
 * Represents a desired state at a given time, used by controllers
 * to compute tracking errors.
 */
struct TrajectoryPoint {
    double x;              // Desired position x [m]
    double y;              // Desired position y [m]
    double v_desired;      // Desired velocity [m/s]
    double theta_desired;  // Desired heading [rad]
    double timestamp;      // Time of this waypoint [s]

    TrajectoryPoint()
        : x(0.0), y(0.0), v_desired(0.0), theta_desired(0.0), timestamp(0.0) {}

    TrajectoryPoint(double x_, double y_, double v_, double theta_, double t_)
        : x(x_), y(y_), v_desired(v_), theta_desired(theta_), timestamp(t_) {}
};

/**
 * @brief Bicycle kinematic model for 2D vehicle motion
 *
 * ## Mathematical Model
 *
 * The bicycle model simplifies a 4-wheeled vehicle to a 2-wheeled bicycle
 * with front-wheel steering. This is a widely-used approximation in robotics
 * and automotive control due to its simplicity and accuracy at low speeds.
 *
 * ### State Equations (Kinematic Bicycle Model)
 *
 * The continuous-time state equations are:
 *
 *   dx/dt = v * cos(θ)           [longitudinal velocity component]
 *   dy/dt = v * sin(θ)           [lateral velocity component]
 *   dθ/dt = (v / L) * tan(δ)     [angular velocity (yaw rate)]
 *   dv/dt = a                    [linear acceleration]
 *
 * where:
 *   - (x, y) = position in global frame [m]
 *   - θ (theta) = heading angle from positive x-axis [rad]
 *   - v = forward velocity [m/s]
 *   - L = wheelbase (distance between front and rear axles) [m]
 *   - δ (delta) = front wheel steering angle [rad]
 *   - a = acceleration command [m/s²]
 *
 * ### Key Assumptions
 *
 * 1. **No slip conditions**: Wheels roll without lateral sliding
 * 2. **Constant wheelbase**: Vehicle geometry doesn't change
 * 3. **Point mass**: Ignores vehicle dimensions (can be extended)
 * 4. **Flat terrain**: No elevation changes (z = 0)
 * 5. **Low speed**: Ignores tire dynamics and weight transfer
 * 6. **Instantaneous steering**: No steering rate limits (can be added)
 *
 * ### Physical Interpretation
 *
 * - The rear wheel always moves in the direction it's pointing (no slip)
 * - The front wheel can be steered at angle δ relative to vehicle body
 * - The instantaneous center of rotation (ICR) is perpendicular to both wheels
 * - Turn radius: R = L / tan(δ)
 *
 * ### Integration Method: Runge-Kutta 4th Order (RK4)
 *
 * We use RK4 instead of simple Euler integration for several reasons:
 *
 * **Euler Method (1st order):**
 *   - Global error: O(dt)
 *   - Requires tiny timesteps (dt ~ 0.001s) for accuracy
 *   - Accumulates drift over long simulations
 *
 * **RK4 Method (4th order):**
 *   - Global error: O(dt^4)
 *   - Allows larger timesteps (dt = 0.01s) with high accuracy
 *   - For dt=0.01s, RK4 is approximately 1000x more accurate
 *   - Industry standard for vehicle simulation
 *
 * **Educational Exercise**: Try replacing RK4 with Euler integration
 * (simply use k1 in step()) and observe the trajectory drift!
 *
 * ### References
 *
 * - Rajamani, R. "Vehicle Dynamics and Control" (2012), Chapter 2
 * - Paden, B. et al. "A Survey of Motion Planning and Control Techniques
 *   for Self-Driving Urban Vehicles" (2016)
 * - Polack, P. et al. "The Kinematic Bicycle Model: A Consistent Model
 *   for Planning Feasible Trajectories for Autonomous Vehicles?" (2017)
 */
class BicycleModel {
public:
    /**
     * @brief Construct a bicycle kinematic model
     *
     * @param wheelbase Distance between front and rear axles [m]
     *                  Typical values: 2.5m (sedan), 3.0m (SUV), 1.5m (small robot)
     * @param dt Integration timestep [s]
     *           Recommended: 0.01s (100Hz) for real-time simulation
     *           Smaller dt increases accuracy but requires more computation
     */
    explicit BicycleModel(double wheelbase = 2.5, double dt = 0.01);

    /**
     * @brief Update vehicle state using RK4 integration
     *
     * This advances the simulation by one timestep (dt) using the
     * 4th-order Runge-Kutta method for numerical integration.
     *
     * @param current_state Current vehicle state
     * @param inputs Control inputs (acceleration, steering angle)
     * @return New vehicle state after time dt
     *
     * @throws std::invalid_argument if inputs contain NaN or Inf
     */
    VehicleState step(const VehicleState& current_state,
                      const ControlInputs& inputs);

    /**
     * @brief Reset model to initial state
     *
     * Useful for starting new simulation runs or testing different
     * initial conditions.
     *
     * @param initial_state Starting state for the simulation
     */
    void reset(const VehicleState& initial_state);

    // Getters

    /**
     * @brief Get current vehicle state
     * @return Const reference to current state (no copy)
     */
    const VehicleState& getCurrentState() const { return state_; }

    /**
     * @brief Get wheelbase parameter
     * @return Wheelbase L [m]
     */
    double getWheelbase() const { return wheelbase_; }

    /**
     * @brief Get integration timestep
     * @return Timestep dt [s]
     */
    double getTimestep() const { return dt_; }

private:
    /**
     * @brief Compute state derivative (right-hand side of ODEs)
     *
     * This implements the kinematic bicycle model equations:
     *   dx/dt = v * cos(θ)
     *   dy/dt = v * sin(θ)
     *   dθ/dt = (v / L) * tan(δ)
     *   dv/dt = a
     *
     * @param state Current state
     * @param inputs Control inputs
     * @return State derivative (rates of change)
     */
    VehicleState computeDerivative(const VehicleState& state,
                                    const ControlInputs& inputs) const;

    /**
     * @brief Add two states (for RK4 intermediate calculations)
     *
     * Helper function: state1 + scale * state2
     * Used in RK4: y_n + k*dt or y_n + k*dt/2
     */
    VehicleState addScaledState(const VehicleState& state1,
                                 const VehicleState& state2,
                                 double scale) const;

    // Model parameters
    double wheelbase_;      // L: Distance between axles [m]
    double dt_;             // Integration timestep [s]

    // Current state (maintained by step() method)
    VehicleState state_;
};

}  // namespace gnc

#endif  // GNC_SIMULATOR_BICYCLE_MODEL_HPP
