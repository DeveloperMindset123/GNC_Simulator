#include "logger.hpp"
#include <spdlog/spdlog.h>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace gnc {

DataLogger::DataLogger(const std::string& output_dir)
    : row_count_(0) {

    // Create output directory if it doesn't exist
    try {
        if (!std::filesystem::exists(output_dir)) {
            spdlog::info("Creating log directory: {}", output_dir);
            std::filesystem::create_directories(output_dir);
        }
    } catch (const std::filesystem::filesystem_error& e) {
        throw std::runtime_error(
            "DataLogger: Failed to create directory '" + output_dir +
            "': " + e.what());
    }

    // Generate timestamped filename
    filename_ = generateFilename(output_dir);

    // Open file for writing
    file_.open(filename_, std::ios::out | std::ios::trunc);

    if (!file_.is_open()) {
        throw std::runtime_error(
            "DataLogger: Failed to open file '" + filename_ + "'");
    }

    spdlog::info("DataLogger initialized: {}", filename_);

    // Write CSV header
    writeHeader();
}

DataLogger::~DataLogger() {
    close();
}

void DataLogger::close() {
    if (file_.is_open()) {
        file_.flush();
        file_.close();
        spdlog::info("DataLogger closed: {} ({} rows written)",
                     filename_, row_count_);
    }
}

void DataLogger::writeHeader() {
    /*
     * CSV Header Format
     * -----------------
     * Columns are organized for easy analysis:
     * 1. Timestamp
     * 2-5. Actual state (x, y, theta, v)
     * 6-9. Reference state (ref_x, ref_y, ref_theta, ref_v)
     * 10-11. Control inputs (steering_cmd, accel_cmd)
     *
     * This allows easy computation of errors:
     *   error_x = ref_x - x
     *   error_v = ref_v - v
     *   etc.
     */

    file_ << "timestamp,"
          << "x,y,theta,v,"
          << "ref_x,ref_y,ref_theta,ref_v,"
          << "steering_cmd,accel_cmd\n";

    // Flush immediately to ensure header is written
    file_.flush();
}

void DataLogger::log(double timestamp,
                     const VehicleState& state,
                     const TrajectoryPoint& reference,
                     const ControlInputs& inputs) {
    /*
     * Write one CSV row with high precision.
     *
     * We use fixed-point notation with sufficient decimal places:
     * - Position: 6 decimals (0.000001 m = 1 micron precision)
     * - Angles: 6 decimals (0.000001 rad ≈ 0.00006 degrees)
     * - Velocity: 4 decimals (0.0001 m/s precision)
     * - Time: 4 decimals (0.0001 s = 0.1 ms precision)
     *
     * This is overkill for visualization but useful for numerical analysis.
     */

    if (!file_.is_open()) {
        spdlog::warn("DataLogger::log called but file is not open");
        return;
    }

    file_ << std::fixed << std::setprecision(4)
          << timestamp << ","
          << std::setprecision(6)
          << state.x << ","
          << state.y << ","
          << state.theta << ","
          << std::setprecision(4)
          << state.v << ","
          << std::setprecision(6)
          << reference.x << ","
          << reference.y << ","
          << reference.theta_desired << ","
          << std::setprecision(4)
          << reference.v_desired << ","
          << std::setprecision(6)
          << inputs.steering_angle << ","
          << std::setprecision(4)
          << inputs.acceleration << "\n";

    row_count_++;

    // Flush every 100 rows to balance performance vs data safety
    // If simulation crashes, we lose at most 100 rows (1 second at 100Hz)
    if (row_count_ % 100 == 0) {
        file_.flush();
    }
}

std::string DataLogger::generateFilename(const std::string& output_dir) {
    /*
     * Generate timestamped filename: gnc_sim_YYYYMMDD_HHMMSS.csv
     *
     * Uses system clock to ensure unique filenames even for rapid
     * successive runs.
     */

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << output_dir << "/gnc_sim_"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << ".csv";

    return ss.str();
}

}  // namespace gnc
