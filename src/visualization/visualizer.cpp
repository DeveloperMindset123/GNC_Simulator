#include "visualization/visualizer.hpp"
#include <spdlog/spdlog.h>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace gnc
{

    Visualizer::Visualizer(int width, int height, double meters_per_pixel)
        : window_(nullptr), renderer_(nullptr),
          width_(width), height_(height),
          meters_per_pixel_(meters_per_pixel),
          last_frame_time_(0)
    {

        // Initialize SDL video subsystem
        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
            throw std::runtime_error(
                "Visualizer: SDL initialization failed: " +
                std::string(SDL_GetError()));
        }

        // Create window
        window_ = SDL_CreateWindow(
            "GNC Simulator - 2D Vehicle Dynamics",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            width, height,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

        if (!window_)
        {
            SDL_Quit();
            throw std::runtime_error(
                "Visualizer: Window creation failed: " +
                std::string(SDL_GetError()));
        }

        // Create renderer with hardware acceleration
        renderer_ = SDL_CreateRenderer(
            window_, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

        if (!renderer_)
        {
            SDL_DestroyWindow(window_);
            SDL_Quit();
            throw std::runtime_error(
                "Visualizer: Renderer creation failed: " +
                std::string(SDL_GetError()));
        }

        // Set blend mode for transparency (if needed later)
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);

        last_frame_time_ = SDL_GetTicks();

        spdlog::info("Visualizer initialized: {}x{} pixels, scale={:.4f} m/px",
                     width, height, meters_per_pixel);
    }

    Visualizer::~Visualizer()
    {
        /*
         * RAII cleanup: Destroy SDL resources in reverse order of creation
         */
        if (renderer_)
        {
            SDL_DestroyRenderer(renderer_);
            renderer_ = nullptr;
        }

        if (window_)
        {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
        }

        SDL_Quit();
        spdlog::info("Visualizer destroyed");
    }

    bool Visualizer::isRunning()
    {
        /*
         * Process SDL events to check for quit conditions:
         * - Window close button clicked
         * - ESC key pressed
         * - System shutdown request
         */
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                return false; // User closed window
            }

            if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.sym == SDLK_ESCAPE)
                {
                    return false; // ESC key pressed
                }
            }
        }

        return true; // Continue running
    }

    void Visualizer::limitFrameRate(int target_fps)
    {
        /*
         * Frame rate limiting using SDL_Delay
         *
         * This prevents the visualization from consuming 100% CPU.
         * The simulation runs at 100Hz, but we only render at 30Hz.
         */
        uint32_t frame_time_ms = 1000 / target_fps; // e.g., 33ms for 30 FPS
        uint32_t current_time = SDL_GetTicks();
        uint32_t elapsed = current_time - last_frame_time_;

        if (elapsed < frame_time_ms)
        {
            SDL_Delay(frame_time_ms - elapsed);
        }

        last_frame_time_ = SDL_GetTicks();
    }

    void Visualizer::render(const VehicleState &vehicle_state,
                            const std::vector<TrajectoryPoint> &reference_trajectory,
                            const std::vector<VehicleState> &actual_path,
                            double velocity_error,
                            double heading_error)
    {
        /*
         * Main rendering function
         * -----------------------
         * 1. Clear screen (white background)
         * 2. Draw reference trajectory (blue)
         * 3. Draw actual path (red)
         * 4. Draw origin marker
         * 5. Draw vehicle (green triangle)
         * 6. Draw text overlay
         * 7. Present frame
         */

        // 1. Clear with white background
        SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255); // White
        SDL_RenderClear(renderer_);

        // 2. Draw reference trajectory (blue)
        if (!reference_trajectory.empty())
        {
            drawTrajectory(reference_trajectory, 0, 0, 255); // Blue
        }

        // 3. Draw actual path (red)
        if (!actual_path.empty())
        {
            drawPath(actual_path, 255, 0, 0); // Red
        }

        // 4. Draw origin marker
        drawOrigin();

        // 5. Draw vehicle (green triangle)
        drawVehicle(vehicle_state);

        // 6. Draw text overlay with metrics
        drawTextOverlay(vehicle_state.timestamp, vehicle_state.v,
                        velocity_error, heading_error);

        // 7. Present rendered frame to screen
        SDL_RenderPresent(renderer_);
    }

    void Visualizer::drawVehicle(const VehicleState &state)
    {
        /*
         * Draw vehicle as oriented triangle
         * ----------------------------------
         * Triangle vertices (in vehicle frame):
         *   Front: (1.5, 0)     [pointing forward]
         *   Rear-left: (-0.5, 0.5)
         *   Rear-right: (-0.5, -0.5)
         *
         * These are rotated by theta and translated to (x, y).
         */

        // Vehicle dimensions in meters (for visualization)
        const double vehicle_length = 1.5;
        const double vehicle_width = 1.0;

        // Triangle vertices in vehicle frame
        double vertices[3][2] = {
            {vehicle_length, 0.0},                    // Front
            {-vehicle_length / 3, vehicle_width / 2}, // Rear-left
            {-vehicle_length / 3, -vehicle_width / 2} // Rear-right
        };

        // Rotate and translate to world frame
        int screen_x[3], screen_y[3];
        for (int i = 0; i < 3; ++i)
        {
            double world_x, world_y;
            rotate2D(vertices[i][0], vertices[i][1], state.theta,
                     world_x, world_y);
            world_x += state.x;
            world_y += state.y;

            worldToScreen(world_x, world_y, screen_x[i], screen_y[i]);
        }

        // Draw filled triangle (green)
        SDL_SetRenderDrawColor(renderer_, 0, 200, 0, 255); // Green

        // Draw three edges
        SDL_RenderDrawLine(renderer_, screen_x[0], screen_y[0],
                           screen_x[1], screen_y[1]);
        SDL_RenderDrawLine(renderer_, screen_x[1], screen_y[1],
                           screen_x[2], screen_y[2]);
        SDL_RenderDrawLine(renderer_, screen_x[2], screen_y[2],
                           screen_x[0], screen_y[0]);

        // Fill triangle (simple scanline fill - good enough for small triangle)
        // For production, could use SDL2_gfx library for filled polygon
        // Here we just draw the outline (good enough for visualization)
    }

    void Visualizer::drawTrajectory(const std::vector<TrajectoryPoint> &trajectory,
                                    uint8_t r, uint8_t g, uint8_t b)
    {
        /*
         * Draw trajectory as connected line segments
         */
        if (trajectory.size() < 2)
            return;

        SDL_SetRenderDrawColor(renderer_, r, g, b, 255);

        for (size_t i = 0; i < trajectory.size() - 1; ++i)
        {
            int x1, y1, x2, y2;
            worldToScreen(trajectory[i].x, trajectory[i].y, x1, y1);
            worldToScreen(trajectory[i + 1].x, trajectory[i + 1].y, x2, y2);

            SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
        }
    }

    void Visualizer::drawPath(const std::vector<VehicleState> &path,
                              uint8_t r, uint8_t g, uint8_t b)
    {
        /*
         * Draw actual vehicle path as connected line segments
         */
        if (path.size() < 2)
            return;

        SDL_SetRenderDrawColor(renderer_, r, g, b, 255);

        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            int x1, y1, x2, y2;
            worldToScreen(path[i].x, path[i].y, x1, y1);
            worldToScreen(path[i + 1].x, path[i + 1].y, x2, y2);

            SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
        }
    }

    void Visualizer::drawOrigin()
    {
        /*
         * Draw crosshairs at (0, 0) in world frame
         * Helps user understand coordinate system and scale
         */
        int origin_x, origin_y;
        worldToScreen(0.0, 0.0, origin_x, origin_y);

        SDL_SetRenderDrawColor(renderer_, 100, 100, 100, 255); // Gray

        // Draw horizontal line
        SDL_RenderDrawLine(renderer_, origin_x - 10, origin_y,
                           origin_x + 10, origin_y);

        // Draw vertical line
        SDL_RenderDrawLine(renderer_, origin_x, origin_y - 10,
                           origin_x, origin_y + 10);
    }

    void Visualizer::drawTextOverlay(double time, double velocity,
                                     double velocity_error, double heading_error)
    {
        /*
         * Draw text overlay with simulation metrics
         *
         * Note: SDL2 doesn't have built-in text rendering.
         * For production, would use SDL_ttf for TrueType fonts.
         *
         * For this educational simulator, we'll skip text rendering to avoid
         * the SDL_ttf dependency. Users can view metrics in CSV logs.
         *
         * Alternative: Render text to console via spdlog::info() every N frames
         */

        // Placeholder: Could integrate SDL_ttf here
        // For now, rely on CSV logs and console output

        static int frame_count = 0;
        frame_count++;

        // Print metrics to console every 30 frames (1 second at 30 FPS)
        if (frame_count % 30 == 0)
        {
            spdlog::info("t={:.1f}s  v={:.2f} m/s  v_err={:.3f}  θ_err={:.3f} rad",
                         time, velocity, velocity_error, heading_error);
        }
    }

    void Visualizer::worldToScreen(double world_x, double world_y,
                                   int &screen_x, int &screen_y) const
    {
        /*
         * Coordinate Transformation
         * -------------------------
         * World frame: Origin at center, +X right, +Y up, units in meters
         * Screen frame: Origin at top-left, +X right, +Y down, units in pixels
         *
         * Transformation:
         * 1. Scale: multiply by (1 / meters_per_pixel)
         * 2. Translate: shift to screen center
         * 3. Flip Y: negate Y to account for inverted screen coordinates
         */

        double scale = 1.0 / meters_per_pixel_;

        // Scale world coordinates to pixels
        double pixel_x = world_x * scale;
        double pixel_y = world_y * scale;

        // Translate to screen center and flip Y
        screen_x = static_cast<int>(pixel_x + width_ / 2);
        screen_y = static_cast<int>(-pixel_y + height_ / 2); // Flip Y
    }

    void Visualizer::rotate2D(double x, double y, double angle,
                              double &out_x, double &out_y) const
    {
        /*
         * 2D Rotation Matrix
         * ------------------
         * [out_x]   [cos(θ)  -sin(θ)] [x]
         * [out_y] = [sin(θ)   cos(θ)] [y]
         */
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);

        out_x = cos_a * x - sin_a * y;
        out_y = sin_a * x + cos_a * y;
    }

} // namespace gnc
