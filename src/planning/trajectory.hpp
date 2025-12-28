#ifndef GNC_SIMULATOR_TRAJECTORY_HPP
#define GNC_SIMULATOR_TRAJECTORY_HPP

#include "dynamics/bicycle_model.hpp"  // For TrajectoryPoint struct
#include <vector>
#include <memory>

namespace gnc {

/**
 * @brief Abstract base class for trajectory generation
 *
 * A trajectory defines the desired path and velocity profile that
 * the vehicle should follow. The controller will compute errors
 * relative to the trajectory and generate control commands to
 * minimize tracking error.
 *
 * ## Design Pattern: Polymorphism
 *
 * This uses object-oriented polymorphism to allow different trajectory
 * types to be used interchangeably:
 *
 *   std::unique_ptr<Trajectory> traj;
 *   traj = std::make_unique<CircleTrajectory>(...);  // or Figure8, etc.
 *   TrajectoryPoint ref = traj->getReference(t);
 *
 * Benefits:
 * - Easy to add new trajectory types without changing existing code
 * - Simulation loop doesn't need to know trajectory implementation details
 * - Can switch trajectories at runtime
 */
class Trajectory {
public:
    virtual ~Trajectory() = default;

    /**
     * @brief Get reference state at given time
     *
     * Returns the desired position, velocity, and heading that the
     * vehicle should have at time t.
     *
     * @param t Simulation time [s]
     * @return Reference trajectory point (x, y, v, theta, timestamp)
     */
    virtual TrajectoryPoint getReference(double t) const = 0;

    /**
     * @brief Pre-generate waypoints for visualization
     *
     * Generates a sequence of trajectory points that can be drawn
     * on the visualization to show the desired path.
     *
     * @param dt Time step between waypoints [s]
     * @param duration Total trajectory duration [s]
     * @return Vector of waypoints covering [0, duration]
     */
    virtual std::vector<TrajectoryPoint> getWaypoints(
        double dt, double duration) const = 0;
};

/**
 * @brief Circular trajectory with constant speed
 *
 * ## Mathematical Description
 *
 * A circle of radius R centered at (cx, cy) can be parametrized as:
 *   x(t) = cx + R * cos(ω*t)
 *   y(t) = cy + R * sin(ω*t)
 *
 * where ω = v / R is the angular velocity for constant speed v.
 *
 * The heading angle θ is always tangent to the circle:
 *   θ(t) = ω*t + π/2  (perpendicular to radius vector)
 *
 * ## Use Cases
 * - Testing steady-state turning behavior
 * - Validating PID controller at constant conditions
 * - Demonstrating bicycle model accuracy (should close the loop)
 *
 * ## Expected Behavior
 * With well-tuned controllers, the vehicle should:
 * - Follow the circle with < 0.1m position error
 * - Maintain constant speed within 0.1 m/s
 * - Return to starting point after one revolution (periodicity check)
 */
class CircleTrajectory : public Trajectory {
public:
    /**
     * @brief Construct a circular trajectory
     *
     * @param radius Circle radius [m] (must be > 0)
     * @param speed Constant forward speed [m/s] (must be > 0)
     * @param center_x Circle center x-coordinate [m]
     * @param center_y Circle center y-coordinate [m]
     *
     * @throws std::invalid_argument if radius or speed <= 0
     */
    CircleTrajectory(double radius, double speed,
                     double center_x = 0.0, double center_y = 0.0);

    TrajectoryPoint getReference(double t) const override;

    std::vector<TrajectoryPoint> getWaypoints(
        double dt, double duration) const override;

    // Getters for trajectory parameters
    double getRadius() const { return radius_; }
    double getSpeed() const { return speed_; }
    double getPeriod() const { return 2.0 * M_PI * radius_ / speed_; }

private:
    double radius_;           // Circle radius [m]
    double speed_;            // Constant forward speed [m/s]
    double center_x_;         // Circle center x [m]
    double center_y_;         // Circle center y [m]
    double angular_velocity_; // ω = v/R [rad/s], precomputed for efficiency
};

/**
 * @brief Figure-8 trajectory (lemniscate)
 *
 * ## Mathematical Description
 *
 * A figure-8 consists of two circles traversed in opposite directions,
 * meeting at a common point with smooth transitions.
 *
 * Implementation: We switch between two circles at the crossover point.
 *
 * ## Challenges
 * - Sharp heading changes at crossover (tests PID robustness)
 * - Requires good derivative control to prevent overshoot
 * - More realistic than simple circle (contains transients)
 *
 * ## Use Cases
 * - Testing controller response to direction changes
 * - Demonstrating path planning with heading discontinuities
 * - Impressive demonstration of tracking capability
 */
class Figure8Trajectory : public Trajectory {
public:
    /**
     * @brief Construct a figure-8 trajectory
     *
     * The figure-8 consists of two circles of given radius, with the
     * crossover point at the origin.
     *
     * @param radius Radius of each circle [m] (must be > 0)
     * @param speed Constant forward speed [m/s] (must be > 0)
     *
     * @throws std::invalid_argument if radius or speed <= 0
     */
    Figure8Trajectory(double radius, double speed);

    TrajectoryPoint getReference(double t) const override;

    std::vector<TrajectoryPoint> getWaypoints(
        double dt, double duration) const override;

private:
    double radius_;
    double speed_;
    double period_;  // Time for one complete figure-8
};

/**
 * @brief Straight line trajectory with smooth acceleration
 *
 * ## Mathematical Description
 *
 * The trajectory consists of three phases:
 * 1. Acceleration phase: v increases from 0 to v_max
 * 2. Constant velocity phase: v = v_max
 * 3. Deceleration phase: v decreases from v_max to 0
 *
 * Position is computed by integrating velocity:
 *   x(t) = x_start + ∫v(τ)dτ * cos(heading)
 *   y(t) = y_start + ∫v(τ)dτ * sin(heading)
 *
 * ## Acceleration Profile
 *
 * We use a smooth S-curve (sinusoidal) for acceleration to avoid
 * discontinuous jerk:
 *   a(t) = a_max * sin(π*t / t_accel)  for 0 < t < t_accel
 *
 * This is more realistic than constant acceleration and easier on actuators.
 *
 * ## Use Cases
 * - Testing velocity PID response to changing setpoints
 * - Validating straight-line motion (no steering errors)
 * - Demonstrating acceleration/deceleration profiles
 */
class StraightLineTrajectory : public Trajectory {
public:
    /**
     * @brief Construct a straight-line trajectory
     *
     * @param start_x Starting x position [m]
     * @param start_y Starting y position [m]
     * @param end_x Ending x position [m]
     * @param end_y Ending y position [m]
     * @param max_speed Maximum velocity during constant phase [m/s]
     * @param accel_time Time to accelerate from 0 to max_speed [s]
     *
     * @throws std::invalid_argument if max_speed <= 0 or accel_time <= 0
     */
    StraightLineTrajectory(double start_x, double start_y,
                           double end_x, double end_y,
                           double max_speed, double accel_time);

    TrajectoryPoint getReference(double t) const override;

    std::vector<TrajectoryPoint> getWaypoints(
        double dt, double duration) const override;

private:
    double start_x_, start_y_;
    double end_x_, end_y_;
    double max_speed_;
    double accel_time_;
    double total_distance_;  // Precomputed: distance from start to end
    double heading_;         // Precomputed: atan2(dy, dx)
};

}  // namespace gnc

#endif  // GNC_SIMULATOR_TRAJECTORY_HPP
