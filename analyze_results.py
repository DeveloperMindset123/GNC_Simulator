#!/usr/bin/env python3
"""
GNC Simulator Results Analyzer

Analyzes CSV log files from the simulator and generates comprehensive
visualizations and performance metrics.

Usage:
    python3 analyze_results.py [path_to_csv_file]

If no file is specified, uses the most recent CSV in build/logs/
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from pathlib import Path


def find_latest_log(logs_dir="build/logs"):
    """Find the most recent CSV log file."""
    log_path = Path(logs_dir)
    if not log_path.exists():
        raise FileNotFoundError(f"Logs directory not found: {logs_dir}")

    csv_files = list(log_path.glob("gnc_sim_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {logs_dir}")

    # Sort by modification time, return newest
    latest = max(csv_files, key=lambda p: p.stat().st_mtime)
    return latest


def load_csv(filename):
    """Load CSV file using numpy."""
    data = np.genfromtxt(filename, delimiter=",", skip_header=1)

    # Column indices based on header:
    # timestamp,x,y,theta,v,ref_x,ref_y,ref_theta,ref_v,steering_cmd,accel_cmd
    columns = {
        "timestamp": 0,
        "x": 1,
        "y": 2,
        "theta": 3,
        "v": 4,
        "ref_x": 5,
        "ref_y": 6,
        "ref_theta": 7,
        "ref_v": 8,
        "steering_cmd": 9,
        "accel_cmd": 10,
    }

    return data, columns


def compute_metrics(data, cols):
    """Compute performance metrics from simulation data."""
    # Position errors
    position_error = np.sqrt(
        (data[:, cols["x"]] - data[:, cols["ref_x"]]) ** 2
        + (data[:, cols["y"]] - data[:, cols["ref_y"]]) ** 2
    )
    position_error_rms = np.sqrt(np.mean(position_error**2))
    position_error_max = position_error.max()
    position_error_mean = position_error.mean()

    # Velocity errors
    velocity_error = data[:, cols["v"]] - data[:, cols["ref_v"]]
    velocity_error_rms = np.sqrt(np.mean(velocity_error**2))
    velocity_error_max = np.abs(velocity_error).max()
    velocity_error_mean = velocity_error.mean()

    # Heading errors (normalized to [-pi, pi])
    heading_error = data[:, cols["theta"]] - data[:, cols["ref_theta"]]
    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
    heading_error_rms = np.sqrt(np.mean(heading_error**2))
    heading_error_max = np.abs(heading_error).max()
    heading_error_mean = heading_error.mean()

    # Control effort
    accel_mean = data[:, cols["accel_cmd"]].mean()
    accel_max = data[:, cols["accel_cmd"]].max()
    accel_min = data[:, cols["accel_cmd"]].min()
    steering_mean = data[:, cols["steering_cmd"]].mean()
    steering_max = data[:, cols["steering_cmd"]].max()

    # Simulation stats
    duration = data[-1, cols["timestamp"]] - data[0, cols["timestamp"]]
    num_samples = len(data)
    avg_frequency = num_samples / duration if duration > 0 else 0

    metrics = {
        "position_error_rms": position_error_rms,
        "position_error_max": position_error_max,
        "position_error_mean": position_error_mean,
        "velocity_error_rms": velocity_error_rms,
        "velocity_error_max": velocity_error_max,
        "velocity_error_mean": velocity_error_mean,
        "heading_error_rms": heading_error_rms,
        "heading_error_max": heading_error_max,
        "heading_error_mean": heading_error_mean,
        "accel_mean": accel_mean,
        "accel_max": accel_max,
        "accel_min": accel_min,
        "steering_mean": steering_mean,
        "steering_max": steering_max,
        "duration": duration,
        "num_samples": num_samples,
        "avg_frequency": avg_frequency,
    }

    return metrics, position_error, velocity_error, heading_error


def print_metrics(metrics):
    """Print formatted metrics to console."""
    print("\n" + "=" * 60)
    print("GNC SIMULATOR PERFORMANCE METRICS")
    print("=" * 60)

    print("\n--- Position Tracking ---")
    print(f"  RMS Error:  {metrics['position_error_rms']:.4f} m")
    print(f"  Mean Error: {metrics['position_error_mean']:.4f} m")
    print(f"  Max Error:  {metrics['position_error_max']:.4f} m")

    print("\n--- Velocity Tracking ---")
    print(f"  RMS Error:  {metrics['velocity_error_rms']:.4f} m/s")
    print(f"  Mean Error: {metrics['velocity_error_mean']:.4f} m/s")
    print(f"  Max Error:  {metrics['velocity_error_max']:.4f} m/s")

    print("\n--- Heading Tracking ---")
    print(
        f"  RMS Error:  {metrics['heading_error_rms']:.4f} rad ({np.rad2deg(metrics['heading_error_rms']):.2f}°)"
    )
    print(
        f"  Mean Error: {metrics['heading_error_mean']:.4f} rad ({np.rad2deg(metrics['heading_error_mean']):.2f}°)"
    )
    print(
        f"  Max Error:  {metrics['heading_error_max']:.4f} rad ({np.rad2deg(metrics['heading_error_max']):.2f}°)"
    )

    print("\n--- Control Effort ---")
    print(f"  Mean Acceleration: {metrics['accel_mean']:.4f} m/s²")
    print(f"  Max Acceleration:  {metrics['accel_max']:.4f} m/s²")
    print(f"  Min Acceleration:  {metrics['accel_min']:.4f} m/s²")
    print(
        f"  Mean Steering:     {metrics['steering_mean']:.4f} rad ({np.rad2deg(metrics['steering_mean']):.2f}°)"
    )
    print(
        f"  Max Steering:      {metrics['steering_max']:.4f} rad ({np.rad2deg(metrics['steering_max']):.2f}°)"
    )

    print("\n--- Simulation Stats ---")
    print(f"  Duration:          {metrics['duration']:.2f} s")
    print(f"  Samples:           {metrics['num_samples']}")
    print(f"  Avg Frequency:     {metrics['avg_frequency']:.1f} Hz")

    print("=" * 60 + "\n")


def plot_results(
    data,
    cols,
    position_error,
    velocity_error,
    heading_error,
    output_file="simulation_results.png",
):
    """Generate comprehensive visualization plots."""
    fig = plt.figure(figsize=(16, 12))

    # 1. Trajectory Plot (top left)
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(
        data[:, cols["ref_x"]],
        data[:, cols["ref_y"]],
        "b--",
        linewidth=2,
        label="Reference",
        alpha=0.7,
    )
    ax1.plot(
        data[:, cols["x"]],
        data[:, cols["y"]],
        "r-",
        linewidth=1,
        label="Actual",
        alpha=0.8,
    )
    ax1.scatter(
        data[0, cols["x"]],
        data[0, cols["y"]],
        c="green",
        s=100,
        marker="o",
        label="Start",
        zorder=5,
    )
    ax1.scatter(
        data[-1, cols["x"]],
        data[-1, cols["y"]],
        c="red",
        s=100,
        marker="X",
        label="End",
        zorder=5,
    )
    ax1.set_xlabel("X Position (m)", fontsize=10)
    ax1.set_ylabel("Y Position (m)", fontsize=10)
    ax1.set_title("2D Trajectory", fontsize=12, fontweight="bold")
    ax1.legend(loc="best", fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")

    # 2. Position Error over Time (top middle)
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(data[:, cols["timestamp"]], position_error, "r-", linewidth=1, alpha=0.7)
    ax2.axhline(
        y=position_error.mean(),
        color="k",
        linestyle="--",
        linewidth=1,
        label=f"Mean: {position_error.mean():.3f} m",
    )
    ax2.set_xlabel("Time (s)", fontsize=10)
    ax2.set_ylabel("Position Error (m)", fontsize=10)
    ax2.set_title("Position Tracking Error", fontsize=12, fontweight="bold")
    ax2.legend(loc="best", fontsize=9)
    ax2.grid(True, alpha=0.3)

    # 3. Velocity Tracking (top right)
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(
        data[:, cols["timestamp"]],
        data[:, cols["ref_v"]],
        "b--",
        linewidth=2,
        label="Reference",
        alpha=0.7,
    )
    ax3.plot(
        data[:, cols["timestamp"]],
        data[:, cols["v"]],
        "r-",
        linewidth=1,
        label="Actual",
        alpha=0.8,
    )
    ax3.set_xlabel("Time (s)", fontsize=10)
    ax3.set_ylabel("Velocity (m/s)", fontsize=10)
    ax3.set_title("Velocity Tracking", fontsize=12, fontweight="bold")
    ax3.legend(loc="best", fontsize=9)
    ax3.grid(True, alpha=0.3)

    # 4. Velocity Error over Time (middle left)
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(data[:, cols["timestamp"]], velocity_error, "r-", linewidth=1, alpha=0.7)
    ax4.axhline(y=0, color="k", linestyle="-", linewidth=0.5)
    ax4.axhline(
        y=velocity_error.mean(),
        color="k",
        linestyle="--",
        linewidth=1,
        label=f"Mean: {velocity_error.mean():.3f} m/s",
    )
    ax4.set_xlabel("Time (s)", fontsize=10)
    ax4.set_ylabel("Velocity Error (m/s)", fontsize=10)
    ax4.set_title("Velocity Error", fontsize=12, fontweight="bold")
    ax4.legend(loc="best", fontsize=9)
    ax4.grid(True, alpha=0.3)

    # 5. Heading Tracking (middle middle)
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(
        data[:, cols["timestamp"]],
        np.rad2deg(data[:, cols["ref_theta"]]),
        "b--",
        linewidth=1,
        label="Reference",
        alpha=0.7,
    )
    ax5.plot(
        data[:, cols["timestamp"]],
        np.rad2deg(data[:, cols["theta"]]),
        "r-",
        linewidth=1,
        label="Actual",
        alpha=0.8,
    )
    ax5.set_xlabel("Time (s)", fontsize=10)
    ax5.set_ylabel("Heading (degrees)", fontsize=10)
    ax5.set_title("Heading Tracking", fontsize=12, fontweight="bold")
    ax5.legend(loc="best", fontsize=9)
    ax5.grid(True, alpha=0.3)

    # 6. Heading Error (middle right)
    ax6 = plt.subplot(3, 3, 6)
    ax6.plot(
        data[:, cols["timestamp"]],
        np.rad2deg(heading_error),
        "r-",
        linewidth=1,
        alpha=0.7,
    )
    ax6.axhline(y=0, color="k", linestyle="-", linewidth=0.5)
    ax6.axhline(
        y=np.rad2deg(heading_error.mean()),
        color="k",
        linestyle="--",
        linewidth=1,
        label=f"Mean: {np.rad2deg(heading_error.mean()):.2f}°",
    )
    ax6.set_xlabel("Time (s)", fontsize=10)
    ax6.set_ylabel("Heading Error (degrees)", fontsize=10)
    ax6.set_title("Heading Error", fontsize=12, fontweight="bold")
    ax6.legend(loc="best", fontsize=9)
    ax6.grid(True, alpha=0.3)

    # 7. Acceleration Command (bottom left)
    ax7 = plt.subplot(3, 3, 7)
    ax7.plot(
        data[:, cols["timestamp"]],
        data[:, cols["accel_cmd"]],
        "g-",
        linewidth=1,
        alpha=0.7,
    )
    ax7.axhline(y=0, color="k", linestyle="-", linewidth=0.5)
    ax7.axhline(
        y=5.0, color="r", linestyle="--", linewidth=1, alpha=0.5, label="Max Limit"
    )
    ax7.axhline(y=-5.0, color="r", linestyle="--", linewidth=1, alpha=0.5)
    ax7.set_xlabel("Time (s)", fontsize=10)
    ax7.set_ylabel("Acceleration (m/s²)", fontsize=10)
    ax7.set_title("Acceleration Command", fontsize=12, fontweight="bold")
    ax7.legend(loc="best", fontsize=9)
    ax7.grid(True, alpha=0.3)

    # 8. Steering Command (bottom middle)
    ax8 = plt.subplot(3, 3, 8)
    ax8.plot(
        data[:, cols["timestamp"]],
        np.rad2deg(data[:, cols["steering_cmd"]]),
        "purple",
        linewidth=1,
        alpha=0.7,
    )
    ax8.axhline(y=0, color="k", linestyle="-", linewidth=0.5)
    ax8.set_xlabel("Time (s)", fontsize=10)
    ax8.set_ylabel("Steering Angle (degrees)", fontsize=10)
    ax8.set_title("Steering Command", fontsize=12, fontweight="bold")
    ax8.grid(True, alpha=0.3)

    # 9. Error Histogram (bottom right)
    ax9 = plt.subplot(3, 3, 9)
    ax9.hist(position_error, bins=50, color="red", alpha=0.6, label="Position Error")
    ax9.axvline(
        x=position_error.mean(),
        color="k",
        linestyle="--",
        linewidth=2,
        label=f"Mean: {position_error.mean():.3f} m",
    )
    ax9.set_xlabel("Position Error (m)", fontsize=10)
    ax9.set_ylabel("Frequency", fontsize=10)
    ax9.set_title("Position Error Distribution", fontsize=12, fontweight="bold")
    ax9.legend(loc="best", fontsize=9)
    ax9.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"\n✓ Visualization saved to: {output_file}")

    return fig


def main():
    # Determine CSV file to analyze
    if len(sys.argv) > 1:
        csv_file = Path(sys.argv[1])
    else:
        csv_file = find_latest_log()

    print(f"\n📊 Analyzing: {csv_file}")

    # Load data
    data, cols = load_csv(csv_file)
    print(f"✓ Loaded {len(data)} data points")

    # Compute metrics
    metrics, position_error, velocity_error, heading_error = compute_metrics(data, cols)

    # Print metrics
    print_metrics(metrics)

    # Generate plots
    output_file = csv_file.parent / f"{csv_file.stem}_analysis.png"
    plot_results(
        data,
        cols,
        position_error,
        velocity_error,
        heading_error,
        output_file=output_file,
    )

    print("\n✓ Analysis complete!")

    # Optional: show plots
    if "--show" in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
