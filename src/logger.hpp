#ifndef GNC_SIMULATOR_LOGGER_HPP
#define GNC_SIMULATOR_LOGGER_HPP

#include "dynamics/bicycle_model.hpp"
#include <fstream>
#include <string>

namespace gnc {

/**
 * @brief CSV data logger for simulation results
 *
 * This logger is separate from spdlog and serves a different purpose:
 * - **spdlog**: Debug messages, warnings, errors (for developers)
 * - **DataLogger**: Numerical data for post-analysis (for users/researchers)
 *
 * ## CSV Format
 *
 * The log file contains one row per simulation timestep with columns:
 *   timestamp, x, y, theta, v, ref_x, ref_y, ref_theta, ref_v,
 *   steering_cmd, accel_cmd
 *
 * This allows offline analysis:
 * - Plot actual vs reference trajectories
 * - Compute tracking errors (RMSE, max error)
 * - Visualize control inputs
 * - Analyze PID performance
 *
 * ## File Naming
 *
 * Files are auto-named with timestamps to prevent overwriting:
 *   gnc_sim_YYYYMMDD_HHMMSS.csv
 *
 * Example: gnc_sim_20250127_143022.csv
 *
 * ## Usage Pattern
 *
 *   DataLogger logger("./logs");
 *   for (each timestep) {
 *       // ... run simulation ...
 *       logger.log(time, state, reference, inputs);
 *   }
 *   logger.close();  // Optional, destructor also closes
 */
class DataLogger {
public:
    /**
     * @brief Create CSV logger with timestamped filename
     *
     * Creates output directory if it doesn't exist.
     *
     * @param output_dir Directory for log files (default: "./logs")
     * @throws std::runtime_error if directory can't be created
     */
    explicit DataLogger(const std::string& output_dir = "./logs");

    /**
     * @brief Destructor ensures file is closed
     */
    ~DataLogger();

    // Non-copyable (file handle can't be safely copied)
    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    /**
     * @brief Log one simulation timestep
     *
     * Writes a CSV row with current state, reference, and control inputs.
     *
     * @param timestamp Simulation time [s]
     * @param state Current vehicle state
     * @param reference Desired state from trajectory
     * @param inputs Control commands (acceleration, steering)
     */
    void log(double timestamp,
             const VehicleState& state,
             const TrajectoryPoint& reference,
             const ControlInputs& inputs);

    /**
     * @brief Flush and close log file
     *
     * Call this at end of simulation to ensure all data is written.
     * Also called automatically by destructor.
     */
    void close();

    /**
     * @brief Get the full path of the log file
     * @return Absolute or relative path to CSV file
     */
    std::string getFilename() const { return filename_; }

    /**
     * @brief Check if logger is ready to write
     * @return true if file is open and healthy
     */
    bool isOpen() const { return file_.is_open(); }

private:
    /**
     * @brief Write CSV header row
     *
     * Called once in constructor after opening file.
     */
    void writeHeader();

    /**
     * @brief Generate timestamped filename
     * @param output_dir Directory path
     * @return Full path like "./logs/gnc_sim_20250127_143022.csv"
     */
    static std::string generateFilename(const std::string& output_dir);

    std::ofstream file_;      // Output file stream (RAII)
    std::string filename_;    // Full path to log file
    size_t row_count_;        // Number of data rows written (for diagnostics)
};

}  // namespace gnc

#endif  // GNC_SIMULATOR_LOGGER_HPP
