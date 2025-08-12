#!/usr/bin/env python3
"""Test script for interactive Task 6 plotting functionality."""

import sys
import numpy as np
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from plot_overlay_interactive import plot_overlay_interactive, create_comparison_dashboard


def generate_test_data():
    """Generate synthetic test data for demonstration."""
    t = np.linspace(0, 100, 1000)
    
    # Generate synthetic trajectory data
    pos_base = np.array([
        10 * np.sin(0.1 * t),
        10 * np.cos(0.1 * t),
        2 * t / 100
    ]).T
    
    vel_base = np.array([
        np.cos(0.1 * t),
        -np.sin(0.1 * t),
        0.02 * np.ones_like(t)
    ]).T
    
    acc_base = np.array([
        -0.1 * np.sin(0.1 * t),
        -0.1 * np.cos(0.1 * t),
        np.zeros_like(t)
    ]).T
    
    # Add noise and variations for different sensors
    noise_scale = 0.1
    
    # IMU data (with more noise)
    pos_imu = pos_base + noise_scale * 2 * np.random.randn(*pos_base.shape)
    vel_imu = vel_base + noise_scale * np.random.randn(*vel_base.shape)
    acc_imu = acc_base + noise_scale * 0.5 * np.random.randn(*acc_base.shape)
    
    # GNSS data (different noise characteristics)
    pos_gnss = pos_base + noise_scale * np.random.randn(*pos_base.shape)
    vel_gnss = vel_base + noise_scale * 0.8 * np.random.randn(*vel_base.shape)
    acc_gnss = acc_base + noise_scale * 0.3 * np.random.randn(*acc_base.shape)
    
    # Fused data (should be between truth and measurements)
    pos_fused = pos_base + noise_scale * 0.5 * np.random.randn(*pos_base.shape)
    vel_fused = vel_base + noise_scale * 0.4 * np.random.randn(*vel_base.shape)
    acc_fused = acc_base + noise_scale * 0.2 * np.random.randn(*acc_base.shape)
    
    # Truth data (clean)
    pos_truth = pos_base
    vel_truth = vel_base
    acc_truth = acc_base
    
    return {
        't_imu': t, 'pos_imu': pos_imu, 'vel_imu': vel_imu, 'acc_imu': acc_imu,
        't_gnss': t[::10], 'pos_gnss': pos_gnss[::10], 'vel_gnss': vel_gnss[::10], 'acc_gnss': acc_gnss[::10],
        't_fused': t, 'pos_fused': pos_fused, 'vel_fused': vel_fused, 'acc_fused': acc_fused,
        't_truth': t, 'pos_truth': pos_truth, 'vel_truth': vel_truth, 'acc_truth': acc_truth,
    }


def test_interactive_plotting():
    """Test the interactive plotting functionality."""
    print("Testing interactive Task 6 plotting...")
    
    # Create output directory
    out_dir = Path("PYTHON/results/test_interactive")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate test data
    data = generate_test_data()
    
    # Test different methods and frames
    methods = ["TRIAD", "Davenport", "SVD"]
    frames = ["NED", "ECEF", "Body"]
    
    for method in methods:
        for frame in frames:
            print(f"  Creating plot for {method} - {frame} frame...")
            
            try:
                plot_overlay_interactive(
                    frame=frame,
                    method=method,
                    **data,
                    out_dir=str(out_dir),
                    filename=f"test_{method}_{frame}_interactive",
                    include_measurements=True,
                    save_static=True,
                )
                print(f"    ✓ Success: {method} - {frame}")
            except Exception as e:
                print(f"    ✗ Failed: {method} - {frame}: {e}")
    
    # Create dashboard
    print("Creating interactive dashboard...")
    try:
        create_comparison_dashboard(out_dir, methods, frames)
        print("    ✓ Dashboard created successfully")
    except Exception as e:
        print(f"    ✗ Dashboard creation failed: {e}")
    
    print(f"\nTest complete! Check results in: {out_dir}")
    print("Open .html files in a web browser to see interactive features.")


if __name__ == "__main__":
    test_interactive_plotting()