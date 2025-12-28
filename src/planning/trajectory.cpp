#include "planning/trajectory.hpp"
#include <stdexcept>
#include <cmath>

namespace gnc {

// ============================================================================
// CircleTrajectory Implementation
// ============================================================================

CircleTrajectory::CircleTrajectory(double radius, double speed,
                                   double center_x, double center_y)
    : radius_(radius), speed_(speed),
      center_x_(center_x), center_y_(center_y) {

    if (radius <= 0.0) {
        throw std::invalid_argument(
            "CircleTrajectory: radius must be positive");
    }
    if (speed <= 0.0) {
        throw std::invalid_argument(
            "CircleTrajectory: speed must be positive");
    }

    // Precompute angular velocity: ω = v / R
    // This is how fast the angle changes for constant speed on circle
    angular_velocity_ = speed / radius;
}

TrajectoryPoint CircleTrajectory::getReference(double t) const {
    /*
     * Parametric Circle Equation
     * ---------------------------
     * x(t) = cx + R*cos(θ(t))
     * y(t) = cy + R*sin(θ(t))
     *
     * where θ(t) = ω*t and ω = v/R
     *
     * The vehicle heading is perpendicular to the radius vector,
     * which means it's tangent to the circle:
     *   heading = θ(t) + π/2
     *
     * Why +π/2?
     * - The radius vector points at angle θ (from center to vehicle)
     * - Velocity is perpendicular to radius (tangent to circle)
     * - Counter-clockwise motion → add π/2
     */

    TrajectoryPoint point;

    // Current angle around circle
    double angle = angular_velocity_ * t;

    // Position on circle
    point.x = center_x_ + radius_ * std::cos(angle);
    point.y = center_y_ + radius_ * std::sin(angle);

    // Heading is tangent to circle (perpendicular to radius)
    point.theta_desired = angle + M_PI / 2.0;

    // Constant speed
    point.v_desired = speed_;

    point.timestamp = t;

    return point;
}

std::vector<TrajectoryPoint> CircleTrajectory::getWaypoints(
    double dt, double duration) const {
    /*
     * Generate waypoints for visualization.
     *
     * For a circle, we want enough points to make it look smooth.
     * dt=0.1s with 60s duration gives 600 points, which is plenty.
     */

    std::vector<TrajectoryPoint> waypoints;
    waypoints.reserve(static_cast<size_t>(duration / dt) + 1);

    for (double t = 0.0; t <= duration; t += dt) {
        waypoints.push_back(getReference(t));
    }

    return waypoints;
}

// ============================================================================
// Figure8Trajectory Implementation
// ============================================================================

Figure8Trajectory::Figure8Trajectory(double radius, double speed)
    : radius_(radius), speed_(speed) {

    if (radius <= 0.0) {
        throw std::invalid_argument(
            "Figure8Trajectory: radius must be positive");
    }
    if (speed <= 0.0) {
        throw std::invalid_argument(
            "Figure8Trajectory: speed must be positive");
    }

    // Period: time to complete one full figure-8
    // Two circles, each with circumference 2πR, traversed at speed v
    period_ = 2.0 * (2.0 * M_PI * radius) / speed;
}

TrajectoryPoint Figure8Trajectory::getReference(double t) const {
    /*
     * Figure-8 Implementation
     * ------------------------
     * We create a figure-8 by alternating between two circles:
     *
     * Circle 1 (right): center at (+radius, 0), counter-clockwise
     * Circle 2 (left):  center at (-radius, 0), counter-clockwise
     *
     * The crossover point is at the origin (0, 0).
     *
     * We switch circles every half-period (one full circle).
     *
     * Alternative Implementation: Lemniscate of Bernoulli
     * For a mathematically smoother figure-8, use:
     *   x(t) = (a * cos(t)) / (1 + sin²(t))
     *   y(t) = (a * sin(t) * cos(t)) / (1 + sin²(t))
     *
     * We use the two-circle approach for simplicity and clarity.
     */

    TrajectoryPoint point;

    // Normalize time to [0, period)
    double t_mod = std::fmod(t, period_);

    // Determine which circle we're on
    double half_period = period_ / 2.0;
    bool on_right_circle = t_mod < half_period;

    // Time within current circle
    double t_circle = on_right_circle ? t_mod : (t_mod - half_period);

    // Angular velocity for each circle
    double omega = speed_ / radius_;
    double angle = omega * t_circle;

    if (on_right_circle) {
        // Right circle: center at (+radius, 0)
        point.x = radius_ + radius_ * std::cos(angle);
        point.y = radius_ * std::sin(angle);
        point.theta_desired = angle + M_PI / 2.0;
    } else {
        // Left circle: center at (-radius, 0)
        // We start at angle π to ensure smooth transition at origin
        double angle_offset = angle + M_PI;
        point.x = -radius_ + radius_ * std::cos(angle_offset);
        point.y = radius_ * std::sin(angle_offset);
        point.theta_desired = angle_offset + M_PI / 2.0;
    }

    point.v_desired = speed_;
    point.timestamp = t;

    return point;
}

std::vector<TrajectoryPoint> Figure8Trajectory::getWaypoints(
    double dt, double duration) const {
    std::vector<TrajectoryPoint> waypoints;
    waypoints.reserve(static_cast<size_t>(duration / dt) + 1);

    for (double t = 0.0; t <= duration; t += dt) {
        waypoints.push_back(getReference(t));
    }

    return waypoints;
}

// ============================================================================
// StraightLineTrajectory Implementation
// ============================================================================

StraightLineTrajectory::StraightLineTrajectory(
    double start_x, double start_y,
    double end_x, double end_y,
    double max_speed, double accel_time)
    : start_x_(start_x), start_y_(start_y),
      end_x_(end_x), end_y_(end_y),
      max_speed_(max_speed), accel_time_(accel_time) {

    if (max_speed <= 0.0) {
        throw std::invalid_argument(
            "StraightLineTrajectory: max_speed must be positive");
    }
    if (accel_time <= 0.0) {
        throw std::invalid_argument(
            "StraightLineTrajectory: accel_time must be positive");
    }

    // Precompute total distance
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    total_distance_ = std::sqrt(dx*dx + dy*dy);

    // Precompute heading (constant for straight line)
    heading_ = std::atan2(dy, dx);
}

TrajectoryPoint StraightLineTrajectory::getReference(double t) const {
    /*
     * Straight Line with Smooth Acceleration Profile
     * -----------------------------------------------
     *
     * Velocity profile (simplified trapezoidal):
     *
     *   v(t) = {
     *     0 → v_max  (accelerating)  for 0 < t < t_accel
     *     v_max      (constant)      for t_accel < t < t_decel
     *     v_max → 0  (decelerating)  for t_decel < t < t_end
     *   }
     *
     * We use sinusoidal acceleration for smoothness:
     *   During accel: v(t) = v_max * (1 - cos(π*t/t_accel)) / 2
     *   This gives: v(0) = 0, v(t_accel) = v_max, smooth derivative
     *
     * Position is computed by integrating velocity.
     *
     * Educational Note: For real robots, you'd also limit jerk (dа/dt)
     * to avoid mechanical stress and oscillations.
     */

    TrajectoryPoint point;

    double v;  // Current velocity
    double distance_traveled;  // How far we've traveled

    if (t < accel_time_) {
        // Phase 1: Accelerating (smooth S-curve)
        // v(t) = v_max * (1 - cos(π*t/t_accel)) / 2
        double phase = t / accel_time_;  // 0 to 1
        v = max_speed_ * (1.0 - std::cos(M_PI * phase)) / 2.0;

        // Integrate velocity to get distance
        // ∫v dt = v_max * (t - (t_accel/π)*sin(π*t/t_accel)) / 2
        distance_traveled = max_speed_ * accel_time_ *
            (phase - std::sin(M_PI * phase) / M_PI) / 2.0;

    } else {
        // Phase 2: Constant velocity
        // Distance during accel phase:
        double accel_distance = max_speed_ * accel_time_ / 2.0;

        // Additional distance at constant speed:
        double const_distance = max_speed_ * (t - accel_time_);

        distance_traveled = accel_distance + const_distance;
        v = max_speed_;

        // TODO: Add deceleration phase if trajectory has defined end time
        // For now, we maintain constant speed indefinitely
    }

    // Compute position along line
    // Clamp to total distance to avoid overshooting endpoint
    if (distance_traveled > total_distance_) {
        distance_traveled = total_distance_;
        v = 0.0;  // Stop at end
    }

    point.x = start_x_ + distance_traveled * std::cos(heading_);
    point.y = start_y_ + distance_traveled * std::sin(heading_);
    point.theta_desired = heading_;  // Constant heading for straight line
    point.v_desired = v;
    point.timestamp = t;

    return point;
}

std::vector<TrajectoryPoint> StraightLineTrajectory::getWaypoints(
    double dt, double duration) const {
    std::vector<TrajectoryPoint> waypoints;
    waypoints.reserve(static_cast<size_t>(duration / dt) + 1);

    for (double t = 0.0; t <= duration; t += dt) {
        waypoints.push_back(getReference(t));
    }

    return waypoints;
}

}  // namespace gnc
