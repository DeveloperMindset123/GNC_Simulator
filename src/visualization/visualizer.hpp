#ifndef GNC_SIMULATOR_VISUALIZER_HPP
#define GNC_SIMULATOR_VISUALIZER_HPP

#include "dynamics/bicycle_model.hpp"
#include <SDL2/SDL.h>
#include <vector>
#include <string>

namespace gnc {

/**
 * @brief Real-time SDL2-based visualization for GNC simulator
 *
 * ## Purpose
 *
 * Provides visual feedback during simulation:
 * - Vehicle position and orientation (as oriented triangle)
 * - Reference trajectory (blue path)
 * - Actual path history (red trail)
 * - Real-time metrics (velocity, errors)
 *
 * ## Design Principles
 *
 * 1. **RAII Resource Management**
 *    - Constructor initializes SDL, creates window/renderer
 *    - Destructor cleans up (SDL_Quit automatically called)
 *    - Non-copyable (SDL resources can't be safely copied)
 *
 * 2. **Coordinate Systems**
 *    - World frame: meters, origin at simulation start point
 *    - Screen frame: pixels, origin at top-left (SDL convention)
 *    - Transformation: scale by meters_per_pixel, flip Y-axis
 *
 * 3. **Performance**
 *    - Throttled to 30 FPS (simulation runs at 100 Hz)
 *    - Only renders when needed (every ~3 sim steps)
 *    - Frame rate limiting prevents excessive CPU usage
 *
 * ## Visualization Elements
 *
 * - **Vehicle**: Oriented triangle showing position and heading
 * - **Reference path**: Blue line from trajectory waypoints
 * - **Actual path**: Red line from vehicle history
 * - **Origin marker**: Crosshairs at (0, 0)
 * - **Text overlay**: Velocity, errors, time
 *
 * ## Controls
 *
 * - Close window or press ESC to stop simulation
 * - Window is resizable (content scales automatically)
 */
class Visualizer {
public:
    /**
     * @brief Construct SDL2 visualizer
     *
     * @param width Window width [pixels] (default: 800)
     * @param height Window height [pixels] (default: 600)
     * @param meters_per_pixel Scale factor for world→screen conversion
     *                         Smaller values zoom in, larger values zoom out
     *                         Default: 0.05 (50 pixels per meter)
     *
     * @throws std::runtime_error if SDL initialization fails
     */
    Visualizer(int width = 800, int height = 600,
               double meters_per_pixel = 0.05);

    /**
     * @brief Destructor cleans up SDL resources
     *
     * Calls SDL_DestroyRenderer, SDL_DestroyWindow, SDL_Quit
     */
    ~Visualizer();

    // Delete copy constructor/assignment (RAII with unique resources)
    Visualizer(const Visualizer&) = delete;
    Visualizer& operator=(const Visualizer&) = delete;

    /**
     * @brief Render current simulation frame
     *
     * Draws all visualization elements and presents to screen.
     *
     * @param vehicle_state Current vehicle state
     * @param reference_trajectory Waypoints to draw (blue)
     * @param actual_path Historical vehicle positions (red)
     * @param velocity_error v_desired - v_actual [m/s]
     * @param heading_error theta_desired - theta_actual [rad]
     */
    void render(const VehicleState& vehicle_state,
                const std::vector<TrajectoryPoint>& reference_trajectory,
                const std::vector<VehicleState>& actual_path,
                double velocity_error,
                double heading_error);

    /**
     * @brief Check if visualization should continue
     *
     * Processes SDL events (window close, keyboard input).
     *
     * @return false if user wants to quit (window closed or ESC pressed)
     */
    bool isRunning();

    /**
     * @brief Limit frame rate to target FPS
     *
     * Sleeps if rendering faster than target to prevent excessive CPU.
     *
     * @param target_fps Desired frame rate (default: 30)
     */
    void limitFrameRate(int target_fps = 30);

private:
    // SDL resources (RAII - cleaned up in destructor)
    SDL_Window* window_;
    SDL_Renderer* renderer_;

    // Display parameters
    int width_;
    int height_;
    double meters_per_pixel_;

    // Frame rate limiting
    uint32_t last_frame_time_;  // SDL_GetTicks() from last render

    // Helper methods

    /**
     * @brief Draw vehicle as oriented triangle
     * @param state Vehicle state (position, heading)
     */
    void drawVehicle(const VehicleState& state);

    /**
     * @brief Draw trajectory waypoints as connected lines
     * @param trajectory Waypoints to draw
     * @param r Red color component [0-255]
     * @param g Green color component [0-255]
     * @param b Blue color component [0-255]
     */
    void drawTrajectory(const std::vector<TrajectoryPoint>& trajectory,
                        uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Draw actual vehicle path as connected lines
     * @param path Historical vehicle states
     * @param r Red color component [0-255]
     * @param g Green color component [0-255]
     * @param b Blue color component [0-255]
     */
    void drawPath(const std::vector<VehicleState>& path,
                  uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Draw origin marker (crosshairs at 0,0)
     */
    void drawOrigin();

    /**
     * @brief Draw text overlay with simulation info
     * @param time Simulation time [s]
     * @param velocity Current velocity [m/s]
     * @param velocity_error Velocity tracking error [m/s]
     * @param heading_error Heading tracking error [rad]
     */
    void drawTextOverlay(double time, double velocity,
                         double velocity_error, double heading_error);

    /**
     * @brief Convert world coordinates to screen coordinates
     *
     * Applies scaling and Y-axis flip (world Y up, screen Y down).
     *
     * @param world_x X position in world frame [m]
     * @param world_y Y position in world frame [m]
     * @param screen_x Output: X pixel coordinate
     * @param screen_y Output: Y pixel coordinate
     */
    void worldToScreen(double world_x, double world_y,
                       int& screen_x, int& screen_y) const;

    /**
     * @brief Rotate a 2D point around origin
     * @param x X coordinate
     * @param y Y coordinate
     * @param angle Rotation angle [rad]
     * @param out_x Output: rotated X
     * @param out_y Output: rotated Y
     */
    void rotate2D(double x, double y, double angle,
                  double& out_x, double& out_y) const;
};

}  // namespace gnc

#endif  // GNC_SIMULATOR_VISUALIZER_HPP
