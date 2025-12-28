/**
 * @file main.cpp
 * @brief Main entry point for GNC Simulator
 *
 * This program demonstrates a complete closed-loop simulation of a
 * 2D autonomous vehicle using:
 * - Bicycle kinematic model with RK4 integration
 * - Dual PID controllers (velocity + steering)
 * - Trajectory generation (circle, figure-8, straight line)
 * - Real-time SDL2 visualization
 * - CSV data logging
 *
 * ## Learning Objectives
 *
 * 1. Understand vehicle dynamics (bicycle model kinematics)
 * 2. Learn PID control theory and tuning
 * 3. Implement real-time simulation loops
 * 4. Visualize control system performance
 * 5. Analyze tracking errors via logged data
 *
 * ## Usage
 *
 *   ./gnc_sim [trajectory_type]
 *
 * Arguments:
 *   trajectory_type: "circle" (default), "figure8", "straight"
 *
 * Examples:
 *   ./gnc_sim              # Run circle trajectory
 *   ./gnc_sim figure8      # Run figure-8 trajectory
 *   ./gnc_sim straight     # Run straight line
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

int main(int argc, char* argv[]) {
    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    // Parse command line arguments
    std::string trajectory_type = "circle";
    if (argc > 1) {
        trajectory_type = argv[1];
    }

    // Configure spdlog
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] [%^%l%$] %v");

    spdlog::info("==============================================");
    spdlog::info("  GNC Simulator - 2D Vehicle Dynamics");
    spdlog::info("  Trajectory: {}", trajectory_type);
    spdlog::info("==============================================");

    try {
        // ====================================================================
        // INITIALIZATION
        // ====================================================================

        // Simulation parameters
        constexpr double SIM_DT = 0.01;         // 100 Hz simulation
        constexpr double SIM_DURATION = 60.0;   // 60 seconds
        constexpr int VIZ_FPS = 30;             // 30 FPS visualization

        spdlog::info("Simulation parameters:");
        spdlog::info("  - Timestep: {:.4f} s ({} Hz)", SIM_DT, int(1.0/SIM_DT));
        spdlog::info("  - Duration: {:.1f} s", SIM_DURATION);
        spdlog::info("  - Visualization: {} FPS", VIZ_FPS);

        // Create dynamics model
        constexpr double WHEELBASE = 2.5;  // meters
        auto dynamics = std::make_unique<BicycleModel>(WHEELBASE, SIM_DT);

        // Initial state: origin, heading east, stationary
        VehicleState initial_state{0.0, 0.0, 0.0, 0.0, 0.0};
        dynamics->reset(initial_state);

        spdlog::info("Vehicle model: Bicycle (L={:.2f} m)", WHEELBASE);

        // Create controllers
        // Velocity PID: Kp=2.0, Ki=0.5, Kd=0.1
        // Output: acceleration in [-5, 5] m/s^2
        auto vel_controller = std::make_unique<PIDController>(
            2.0, 0.5, 0.1,      // Kp, Ki, Kd
            -5.0, 5.0,          // output min/max
            10.0                // integral max (anti-windup)
        );

        // Steering PID: Kp=1.5, Ki=0.0, Kd=0.3
        // Output: steering angle in [-0.5, 0.5] rad (≈ ±29°)
        // Note: Ki=0 to avoid integral windup during sharp turns
        auto steer_controller = std::make_unique<PIDController>(
            1.5, 0.0, 0.3,      // Kp, Ki, Kd
            -0.5, 0.5,          // output min/max
            5.0                 // integral max
        );

        spdlog::info("Controllers: Dual PID (velocity + steering)");
        spdlog::info("  - Velocity PID: Kp=2.0, Ki=0.5, Kd=0.1");
        spdlog::info("  - Steering PID: Kp=1.5, Ki=0.0, Kd=0.3");

        // Create trajectory based on command line argument
        std::unique_ptr<Trajectory> trajectory;

        if (trajectory_type == "circle") {
            trajectory = std::make_unique<CircleTrajectory>(
                10.0,   // radius [m]
                5.0,    // speed [m/s]
                0.0, 0.0 // center (x, y)
            );
            spdlog::info("Trajectory: Circle (R=10m, v=5m/s)");

        } else if (trajectory_type == "figure8") {
            trajectory = std::make_unique<Figure8Trajectory>(
                10.0,   // radius [m]
                5.0     // speed [m/s]
            );
            spdlog::info("Trajectory: Figure-8 (R=10m, v=5m/s)");

        } else if (trajectory_type == "straight") {
            trajectory = std::make_unique<StraightLineTrajectory>(
                0.0, 0.0,   // start (x, y)
                50.0, 0.0,  // end (x, y)
                8.0,        // max speed [m/s]
                3.0         // accel time [s]
            );
            spdlog::info("Trajectory: Straight line (50m, v_max=8m/s)");

        } else {
            spdlog::error("Unknown trajectory type: {}", trajectory_type);
            spdlog::error("Valid options: circle, figure8, straight");
            return 1;
        }

        // Pre-generate waypoints for visualization
        auto waypoints = trajectory->getWaypoints(0.1, SIM_DURATION);
        spdlog::info("Generated {} waypoints for visualization",
                     waypoints.size());

        // Create visualizer
        auto visualizer = std::make_unique<Visualizer>(
            800, 600,   // window size
            0.05        // meters per pixel (zoom level)
        );

        // Create data logger
        auto logger = std::make_unique<DataLogger>("./logs");

        // Path history (for visualization)
        std::vector<VehicleState> path_history;
        path_history.reserve(10000);  // Pre-allocate for 100 seconds

        spdlog::info("Initialization complete. Starting simulation...\n");

        // ====================================================================
        // SIMULATION LOOP
        // ====================================================================

        double sim_time = 0.0;
        int sim_step = 0;
        const int total_steps = static_cast<int>(SIM_DURATION / SIM_DT);

        while (sim_time < SIM_DURATION && visualizer->isRunning()) {

            // 1. Get reference from trajectory
            TrajectoryPoint reference = trajectory->getReference(sim_time);

            // 2. Get current state
            VehicleState current_state = dynamics->getCurrentState();

            // 3. Compute tracking errors

            // Velocity error (straightforward)
            double velocity_error = reference.v_desired - current_state.v;

            // Heading error (normalize to [-π, π] for shortest rotation)
            double heading_error = reference.theta_desired - current_state.theta;

            // Angle normalization: wrap to [-π, π]
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

            // Position error (not used for control, but useful for logging)
            double position_error = std::sqrt(
                std::pow(reference.x - current_state.x, 2) +
                std::pow(reference.y - current_state.y, 2)
            );

            // 4. Compute control inputs using PID controllers

            ControlInputs inputs;
            inputs.acceleration = vel_controller->compute(velocity_error, SIM_DT);
            inputs.steering_angle = steer_controller->compute(heading_error, SIM_DT);

            // 5. Step dynamics forward (RK4 integration)
            VehicleState new_state = dynamics->step(current_state, inputs);

            // 6. Update path history for visualization
            path_history.push_back(new_state);

            // Limit history size to prevent unbounded growth
            if (path_history.size() > 10000) {
                path_history.erase(path_history.begin());
            }

            // 7. Log data to CSV
            logger->log(sim_time, new_state, reference, inputs);

            // 8. Visualization (throttled to VIZ_FPS)
            // Render approximately every (100Hz / 30Hz) ≈ 3 steps
            if (sim_step % 3 == 0) {
                visualizer->render(new_state, waypoints, path_history,
                                   velocity_error, heading_error);
                visualizer->limitFrameRate(VIZ_FPS);
            }

            // 9. Progress reporting (every 10 seconds)
            if (sim_step % 1000 == 0 && sim_step > 0) {
                spdlog::info("Progress: {:.0f}% ({:.1f}/{:.1f}s)  pos_err={:.3f}m",
                             100.0 * sim_time / SIM_DURATION,
                             sim_time, SIM_DURATION, position_error);
            }

            // Advance simulation time
            sim_time += SIM_DT;
            sim_step++;
        }

        // ====================================================================
        // CLEANUP AND SUMMARY
        // ====================================================================

        logger->close();

        spdlog::info("\n==============================================");
        spdlog::info("Simulation complete!");
        spdlog::info("  - Duration: {:.2f} s", sim_time);
        spdlog::info("  - Steps: {}", sim_step);
        spdlog::info("  - Log file: {}", logger->getFilename());
        spdlog::info("==============================================");

        spdlog::info("\nPost-Analysis Tips:");
        spdlog::info("1. Load CSV in Python/MATLAB for error analysis");
        spdlog::info("2. Plot actual vs reference trajectories");
        spdlog::info("3. Compute RMSE for position and velocity");
        spdlog::info("4. Visualize control inputs (steering, accel)");
        spdlog::info("5. Experiment with different PID gains!");

    } catch (const std::exception& e) {
        spdlog::error("Fatal error: {}", e.what());
        spdlog::error("Simulation aborted.");
        return 1;
    }

    return 0;
}
