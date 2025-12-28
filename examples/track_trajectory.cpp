/**
 * @file track_trajectory.cpp
 * @brief Example program demonstrating figure-8 trajectory tracking
 *
 * This example shows how to use the GNC simulator components to create
 * a custom simulation with different configurations and parameters.
 *
 * ## Features Demonstrated
 * - Custom PID tuning for aggressive tracking
 * - Figure-8 trajectory (more challenging than circle)
 * - Higher simulation frequency (200 Hz)
 * - Data logging and visualization
 *
 * ## Usage
 *   ./track_trajectory
 */

#include <memory>
#include <iostream>
#include <spdlog/spdlog.h>

#include "dynamics/bicycle_model.hpp"
#include "control/pid_controller.hpp"
#include "planning/trajectory.hpp"
#include "visualization/visualizer.hpp"
#include "logger.hpp"

using namespace gnc;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %v");

    spdlog::info("============================================");
    spdlog::info("  Figure-8 Trajectory Tracking Example");
    spdlog::info("============================================\n");

    try {
        // Configuration
        constexpr double SIM_DT = 0.005;        // 200 Hz (higher frequency)
        constexpr double SIM_DURATION = 40.0;   // 40 seconds
        constexpr int VIZ_FPS = 30;

        spdlog::info("Simulation: {:.3f}s timestep ({} Hz), {:.0f}s duration",
                     SIM_DT, int(1.0/SIM_DT), SIM_DURATION);

        // Create vehicle dynamics
        auto dynamics = std::make_unique<BicycleModel>(2.5, SIM_DT);

        // Start at a point on the figure-8 for smooth entry
        VehicleState initial{10.0, 0.0, M_PI/2, 5.0, 0.0};
        dynamics->reset(initial);

        spdlog::info("Initial state: x={:.1f}m, y={:.1f}m, θ={:.2f}rad, v={:.1f}m/s",
                     initial.x, initial.y, initial.theta, initial.v);

        // Create controllers with more aggressive tuning
        // Higher Kp for faster response, higher Kd for damping
        auto vel_controller = std::make_unique<PIDController>(
            3.0, 0.8, 0.2,      // Kp, Ki, Kd (more aggressive)
            -6.0, 6.0,          // output limits
            15.0                // integral max
        );

        auto steer_controller = std::make_unique<PIDController>(
            2.0, 0.0, 0.5,      // Higher Kp and Kd for sharp turns
            -0.6, 0.6,          // Allow larger steering angles
            10.0
        );

        spdlog::info("Controllers: Aggressive tuning for figure-8");
        spdlog::info("  Velocity PID: Kp=3.0, Ki=0.8, Kd=0.2");
        spdlog::info("  Steering PID: Kp=2.0, Ki=0.0, Kd=0.5\n");

        // Create figure-8 trajectory
        auto trajectory = std::make_unique<Figure8Trajectory>(
            10.0,   // radius [m]
            5.0     // speed [m/s]
        );

        spdlog::info("Trajectory: Figure-8 (R=10m, v=5m/s)");

        // Pre-generate waypoints
        auto waypoints = trajectory->getWaypoints(0.1, SIM_DURATION);
        spdlog::info("Waypoints: {}\n", waypoints.size());

        // Create visualizer with tighter zoom
        auto visualizer = std::make_unique<Visualizer>(
            1000, 800,  // Larger window
            0.04        // Tighter zoom (25 pixels per meter)
        );

        // Create logger
        auto logger = std::make_unique<DataLogger>("./logs");

        // Path history
        std::vector<VehicleState> path_history;
        path_history.reserve(20000);

        spdlog::info("Starting simulation...\n");

        // Simulation loop
        double sim_time = 0.0;
        int sim_step = 0;
        double max_position_error = 0.0;
        double max_velocity_error = 0.0;

        while (sim_time < SIM_DURATION && visualizer->isRunning()) {

            // Get reference
            TrajectoryPoint reference = trajectory->getReference(sim_time);

            // Get current state
            VehicleState current_state = dynamics->getCurrentState();

            // Compute errors
            double velocity_error = reference.v_desired - current_state.v;

            double heading_error = reference.theta_desired - current_state.theta;
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

            double position_error = std::sqrt(
                std::pow(reference.x - current_state.x, 2) +
                std::pow(reference.y - current_state.y, 2)
            );

            // Track max errors
            max_position_error = std::max(max_position_error, position_error);
            max_velocity_error = std::max(max_velocity_error, std::abs(velocity_error));

            // Compute controls
            ControlInputs inputs;
            inputs.acceleration = vel_controller->compute(velocity_error, SIM_DT);
            inputs.steering_angle = steer_controller->compute(heading_error, SIM_DT);

            // Step dynamics
            VehicleState new_state = dynamics->step(current_state, inputs);

            // Update path history
            path_history.push_back(new_state);
            if (path_history.size() > 20000) {
                path_history.erase(path_history.begin());
            }

            // Log data
            logger->log(sim_time, new_state, reference, inputs);

            // Visualization (every ~7 steps for 200Hz → 30Hz)
            if (sim_step % 7 == 0) {
                visualizer->render(new_state, waypoints, path_history,
                                   velocity_error, heading_error);
                visualizer->limitFrameRate(VIZ_FPS);
            }

            // Progress (every 5 seconds)
            if (sim_step % 1000 == 0 && sim_step > 0) {
                spdlog::info("t={:.1f}s  pos_err={:.3f}m  v_err={:.3f}m/s  max_pos_err={:.3f}m",
                             sim_time, position_error, velocity_error, max_position_error);
            }

            sim_time += SIM_DT;
            sim_step++;
        }

        logger->close();

        spdlog::info("\n============================================");
        spdlog::info("Simulation Complete!");
        spdlog::info("  Duration: {:.2f}s ({} steps)", sim_time, sim_step);
        spdlog::info("  Max position error: {:.4f}m", max_position_error);
        spdlog::info("  Max velocity error: {:.4f}m/s", max_velocity_error);
        spdlog::info("  Log: {}", logger->getFilename());
        spdlog::info("============================================\n");

        // Performance evaluation
        if (max_position_error < 0.5) {
            spdlog::info("Performance: EXCELLENT (< 0.5m error)");
        } else if (max_position_error < 1.0) {
            spdlog::info("Performance: GOOD (< 1.0m error)");
        } else {
            spdlog::info("Performance: NEEDS TUNING (> 1.0m error)");
            spdlog::info("Try adjusting PID gains in the source code!");
        }

    } catch (const std::exception& e) {
        spdlog::error("Error: {}", e.what());
        return 1;
    }

    return 0;
}
