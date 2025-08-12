#!/usr/bin/env python3
"""Test interactive Task 6 plotting with existing pipeline results."""

import sys
import numpy as np
from pathlib import Path
import scipy.io

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from plot_overlay_interactive import plot_overlay_interactive, create_comparison_dashboard


def load_and_test_real_data():
    """Load real pipeline results and test interactive plotting."""
    print("Testing interactive plotting with real IMU/GNSS fusion results...")
    
    # Look for existing result files
    results_dir = Path("PYTHON/results")
    kf_files = list(results_dir.glob("*_kf_output.mat"))
    
    if not kf_files:
        print("No kalman filter output files found. Run the pipeline first.")
        return
    
    kf_file = kf_files[0]  # Use the first available file
    print(f"Using result file: {kf_file}")
    
    # Load the data
    try:
        data = scipy.io.loadmat(str(kf_file))
        print("Available keys in MAT file:")
        for key in sorted(data.keys()):
            if not key.startswith('__'):
                print(f"  {key}: {data[key].shape if hasattr(data[key], 'shape') else type(data[key])}")
    except Exception as e:
        print(f"Error loading MAT file: {e}")
        return
    
    # Extract the required data (handle different possible key names)
    time_keys = ['time_s', 'time', 't']
    pos_keys = ['pos_ecef_m', 'pos_ecef', 'position_ecef']
    vel_keys = ['vel_ecef_ms', 'vel_ecef', 'velocity_ecef']
    
    time_data = None
    for key in time_keys:
        if key in data:
            time_data = data[key].flatten()
            break
    
    pos_data = None
    for key in pos_keys:
        if key in data:
            pos_data = data[key]
            if pos_data.shape[1] == 3:  # Ensure it's Nx3
                break
            elif pos_data.shape[0] == 3:  # If it's 3xN, transpose
                pos_data = pos_data.T
                break
    
    vel_data = None
    for key in vel_keys:
        if key in data:
            vel_data = data[key]
            if vel_data.shape[1] == 3:  # Ensure it's Nx3
                break
            elif vel_data.shape[0] == 3:  # If it's 3xN, transpose
                vel_data = vel_data.T
                break
    
    if time_data is None or pos_data is None or vel_data is None:
        print("Required data not found in MAT file")
        return
    
    # Generate synthetic acceleration and other sensor data for demonstration
    print(f"Data shapes - time: {time_data.shape}, pos: {pos_data.shape}, vel: {vel_data.shape}")
    
    # Compute acceleration from velocity
    dt = np.mean(np.diff(time_data))
    acc_data = np.gradient(vel_data, dt, axis=0)
    
    # Create synthetic IMU and GNSS data by adding noise
    np.random.seed(42)  # For reproducible results
    noise_scale = 0.1
    
    # IMU data (higher frequency, more noise)
    time_imu = time_data
    pos_imu = pos_data + noise_scale * 2 * np.random.randn(*pos_data.shape)
    vel_imu = vel_data + noise_scale * np.random.randn(*vel_data.shape)
    acc_imu = acc_data + noise_scale * 0.5 * np.random.randn(*acc_data.shape)
    
    # GNSS data (lower frequency, different noise)
    time_gnss = time_data[::10]  # Subsample for GNSS
    pos_gnss = pos_data[::10] + noise_scale * np.random.randn(len(time_gnss), 3)
    vel_gnss = vel_data[::10] + noise_scale * 0.8 * np.random.randn(len(time_gnss), 3)
    acc_gnss = acc_data[::10] + noise_scale * 0.3 * np.random.randn(len(time_gnss), 3)
    
    # Fused data (the actual pipeline output)
    time_fused = time_data
    pos_fused = pos_data
    vel_fused = vel_data
    acc_fused = acc_data
    
    # Truth data (cleaner version)
    time_truth = time_data
    pos_truth = pos_data + noise_scale * 0.1 * np.random.randn(*pos_data.shape)
    vel_truth = vel_data + noise_scale * 0.05 * np.random.randn(*vel_data.shape)
    acc_truth = acc_data + noise_scale * 0.02 * np.random.randn(*acc_data.shape)
    
    # Create output directory
    out_dir = results_dir / "task6_interactive_real"
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Test different frames
    frames = ["ECEF", "NED", "Body"]
    method = "TRIAD"
    
    for frame in frames:
        print(f"  Creating interactive plot for {method} - {frame} frame...")
        
        try:
            plot_overlay_interactive(
                frame=frame,
                method=method,
                t_imu=time_imu,
                pos_imu=pos_imu,
                vel_imu=vel_imu,
                acc_imu=acc_imu,
                t_gnss=time_gnss,
                pos_gnss=pos_gnss,
                vel_gnss=vel_gnss,
                acc_gnss=acc_gnss,
                t_fused=time_fused,
                pos_fused=pos_fused,
                vel_fused=vel_fused,
                acc_fused=acc_fused,
                out_dir=str(out_dir),
                t_truth=time_truth,
                pos_truth=pos_truth,
                vel_truth=vel_truth,
                acc_truth=acc_truth,
                filename=f"real_data_{method}_{frame}_interactive",
                include_measurements=True,
                save_static=True,
            )
            print(f"    ✓ Success: {method} - {frame}")
        except Exception as e:
            print(f"    ✗ Failed: {method} - {frame}: {e}")
            import traceback
            traceback.print_exc()
    
    # Create dashboard
    print("Creating interactive dashboard...")
    try:
        create_comparison_dashboard(out_dir, [method], frames)
        print("    ✓ Dashboard created successfully")
    except Exception as e:
        print(f"    ✗ Dashboard creation failed: {e}")
    
    print(f"\nReal data test complete! Check results in: {out_dir}")
    print("Features of the interactive plots:")
    print("  • Zoom and pan on any subplot")
    print("  • Hover tooltips showing exact values")
    print("  • Legend toggling to show/hide data series")
    print("  • Export tools built into the plot interface")


if __name__ == "__main__":
    load_and_test_real_data()