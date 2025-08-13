#!/usr/bin/env python3
"""Test interactive Task 6 plotting with subsampled data for better performance."""

import sys
import numpy as np
from pathlib import Path
import scipy.io

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from plot_overlay_interactive import plot_overlay_interactive, create_comparison_dashboard


def subsample_data(time_data, *data_arrays, max_points=5000):
    """Subsample data arrays to reduce size for interactive plotting."""
    if len(time_data) <= max_points:
        return time_data, *data_arrays
    
    # Create indices for subsampling
    indices = np.linspace(0, len(time_data) - 1, max_points, dtype=int)
    
    # Subsample all arrays
    subsampled = [time_data[indices]]
    for data_array in data_arrays:
        if data_array is not None:
            subsampled.append(data_array[indices])
        else:
            subsampled.append(None)
    
    return tuple(subsampled)


def test_optimized_interactive():
    """Test interactive plotting with optimized data size."""
    print("Testing optimized interactive plotting with real IMU/GNSS fusion results...")
    
    # Look for existing result files
    results_dir = Path("PYTHON/results")
    kf_files = list(results_dir.glob("*_kf_output.mat"))
    
    if not kf_files:
        print("No kalman filter output files found. Run the pipeline first.")
        return
    
    kf_file = kf_files[0]
    print(f"Using result file: {kf_file}")
    
    # Load the data
    try:
        data = scipy.io.loadmat(str(kf_file))
    except Exception as e:
        print(f"Error loading MAT file: {e}")
        return
    
    # Extract data
    time_data = data['time_s'].flatten()
    pos_data = data['pos_ecef_m']
    vel_data = data['vel_ecef_ms']
    
    print(f"Original data size: {len(time_data)} points")
    
    # Compute acceleration
    dt = np.mean(np.diff(time_data))
    acc_data = np.gradient(vel_data, dt, axis=0)
    
    # Subsample for better performance
    max_points = 2000  # Reasonable size for interactive plotting
    time_sub, pos_sub, vel_sub, acc_sub = subsample_data(
        time_data, pos_data, vel_data, acc_data, max_points=max_points
    )
    
    print(f"Subsampled to: {len(time_sub)} points for better performance")
    
    # Create synthetic sensor data
    np.random.seed(42)
    noise_scale = 0.1
    
    # IMU data (more noise)
    pos_imu = pos_sub + noise_scale * 2 * np.random.randn(*pos_sub.shape)
    vel_imu = vel_sub + noise_scale * np.random.randn(*vel_sub.shape) 
    acc_imu = acc_sub + noise_scale * 0.5 * np.random.randn(*acc_sub.shape)
    
    # GNSS data (different sampling and noise)
    gnss_indices = np.arange(0, len(time_sub), 10)  # Every 10th point
    time_gnss = time_sub[gnss_indices]
    pos_gnss = pos_sub[gnss_indices] + noise_scale * np.random.randn(len(gnss_indices), 3)
    vel_gnss = vel_sub[gnss_indices] + noise_scale * 0.8 * np.random.randn(len(gnss_indices), 3)
    acc_gnss = acc_sub[gnss_indices] + noise_scale * 0.3 * np.random.randn(len(gnss_indices), 3)
    
    # Truth data (slight noise)
    pos_truth = pos_sub + noise_scale * 0.1 * np.random.randn(*pos_sub.shape)
    vel_truth = vel_sub + noise_scale * 0.05 * np.random.randn(*vel_sub.shape)
    acc_truth = acc_sub + noise_scale * 0.02 * np.random.randn(*acc_sub.shape)
    
    # Create output directory
    out_dir = results_dir / "task6_interactive_optimized"
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Test all methods and frames
    methods = ["TRIAD", "Davenport", "SVD"]
    frames = ["ECEF", "NED", "Body"]
    
    for method in methods:
        for frame in frames:
            print(f"  Creating interactive plot for {method} - {frame} frame...")
            
            try:
                plot_overlay_interactive(
                    frame=frame,
                    method=method,
                    t_imu=time_sub,
                    pos_imu=pos_imu,
                    vel_imu=vel_imu,
                    acc_imu=acc_imu,
                    t_gnss=time_gnss,
                    pos_gnss=pos_gnss,
                    vel_gnss=vel_gnss,
                    acc_gnss=acc_gnss,
                    t_fused=time_sub,
                    pos_fused=pos_sub,
                    vel_fused=vel_sub,
                    acc_fused=acc_sub,
                    out_dir=str(out_dir),
                    t_truth=time_sub,
                    pos_truth=pos_truth,
                    vel_truth=vel_truth,
                    acc_truth=acc_truth,
                    filename=f"optimized_{method}_{frame}_interactive",
                    include_measurements=True,
                    save_static=True,
                )
                print(f"    ✓ Success: {method} - {frame}")
            except Exception as e:
                print(f"    ✗ Failed: {method} - {frame}: {e}")
    
    # Create comprehensive dashboard
    print("Creating interactive dashboard...")
    try:
        create_comparison_dashboard(out_dir, methods, frames)
        print("    ✓ Dashboard created successfully")
    except Exception as e:
        print(f"    ✗ Dashboard creation failed: {e}")
    
    # Show file sizes for comparison
    print(f"\nResults in: {out_dir}")
    html_files = list(out_dir.glob("*.html"))
    print("Generated files:")
    for html_file in html_files:
        size_mb = html_file.stat().st_size / (1024 * 1024)
        print(f"  {html_file.name}: {size_mb:.1f} MB")
    
    print(f"\nInteractive features available:")
    print("  • Zoom: Click and drag to zoom into regions of interest")
    print("  • Pan: Hold shift and drag to pan across the data")
    print("  • Hover: Mouse over data points for exact values and timestamps")
    print("  • Legend: Click legend items to show/hide data series")
    print("  • Export: Use toolbar to export plots as PNG/PDF")
    print("  • Reset: Double-click to reset zoom/pan")


if __name__ == "__main__":
    test_optimized_interactive()