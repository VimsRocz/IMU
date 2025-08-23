#!/usr/bin/env python3
"""
Quick demonstration of IMU vibration simulation and compensation capabilities.

This script provides a simple demonstration of the vibration analysis features
without requiring the full examples to run.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import os
import sys

# Add source directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'PYTHON', 'src'))

try:
    from vibration_model import VibrationModel, create_quadrotor_vibration_model
    from vibration_compensation import detect_vibration_simple, compensate_vibration_simple
    print("✓ Vibration modules imported successfully")
except ImportError as e:
    print(f"✗ Failed to import vibration modules: {e}")
    print("Please ensure the vibration modules are in PYTHON/src/")
    sys.exit(1)


def quick_demo():
    """Run a quick demonstration of vibration capabilities."""
    print("IMU Vibration Simulation and Compensation - Quick Demo")
    print("=" * 55)
    
    # Parameters
    fs = 400.0      # Sampling frequency
    duration = 5.0  # Duration in seconds
    
    print(f"Simulating {duration} seconds of IMU data at {fs} Hz...")
    
    # Generate clean IMU motion (low frequency)
    t = np.arange(0, duration, 1/fs)
    clean_accel = np.column_stack([
        0.5 * np.sin(2 * np.pi * 0.2 * t),    # X: 0.2 Hz
        0.3 * np.cos(2 * np.pi * 0.3 * t),    # Y: 0.3 Hz  
        9.81 + 0.4 * np.sin(2 * np.pi * 0.1 * t)  # Z: gravity + 0.1 Hz
    ])
    
    # Generate quadrotor vibration
    vibration_model = create_quadrotor_vibration_model(
        fs=fs, 
        rotor_frequency=100.0,
        amplitude_acc=1.5,
        amplitude_gyro=0.15
    )
    
    vibration_signals = vibration_model.generate_vibration(
        duration, **vibration_model.default_params
    )
    
    # Apply vibration to clean signal
    vibrated_accel = clean_accel + vibration_signals['accel_vib']
    
    print("✓ Generated clean and vibrated accelerometer data")
    
    # Test vibration detection
    vibration_detected = detect_vibration_simple(vibrated_accel, fs=fs, method='statistical')
    print(f"✓ Vibration detection result: {vibration_detected}")
    
    # Test vibration compensation
    compensated_accel = compensate_vibration_simple(vibrated_accel, fs=fs, cutoff_freq=15.0)
    print("✓ Applied vibration compensation")
    
    # Calculate metrics
    error_before = np.sqrt(np.mean((vibrated_accel - clean_accel)**2, axis=0))
    error_after = np.sqrt(np.mean((compensated_accel - clean_accel)**2, axis=0))
    improvement = (error_before - error_after) / error_before * 100
    
    print(f"\nRMSE Improvement by Axis:")
    print(f"  X-axis: {improvement[0]:.1f}%")
    print(f"  Y-axis: {improvement[1]:.1f}%") 
    print(f"  Z-axis: {improvement[2]:.1f}%")
    print(f"  Average: {np.mean(improvement):.1f}%")
    
    # Create simple plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('IMU Vibration Analysis Quick Demo', fontsize=14, fontweight='bold')
    
    # Plot 1: Original signals
    ax = axes[0, 0]
    ax.plot(t, clean_accel[:, 2], 'g-', linewidth=2, label='Clean Signal')
    ax.plot(t, vibrated_accel[:, 2], 'r-', linewidth=1, alpha=0.8, label='With Vibration')
    ax.set_title('Accelerometer Z-axis: Clean vs Vibrated')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Compensation result
    ax = axes[0, 1]
    ax.plot(t, clean_accel[:, 2], 'g-', linewidth=2, label='True Signal')
    ax.plot(t, compensated_accel[:, 2], 'b-', linewidth=1.5, alpha=0.9, label='Compensated')
    ax.set_title('Vibration Compensation Result')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Vibration signal
    ax = axes[1, 0]
    ax.plot(t, vibration_signals['accel_vib'][:, 0], 'r-', linewidth=1, label='X')
    ax.plot(t, vibration_signals['accel_vib'][:, 1], 'g-', linewidth=1, label='Y')
    ax.plot(t, vibration_signals['accel_vib'][:, 2], 'b-', linewidth=1, label='Z')
    ax.set_title('Generated Quadrotor Vibration')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Vibration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Error comparison
    ax = axes[1, 1]
    axes_labels = ['X', 'Y', 'Z']
    x_pos = np.arange(len(axes_labels))
    
    bar_width = 0.35
    ax.bar(x_pos - bar_width/2, error_before, bar_width, label='Before Compensation', 
           color='red', alpha=0.7)
    ax.bar(x_pos + bar_width/2, error_after, bar_width, label='After Compensation', 
           color='blue', alpha=0.7)
    
    ax.set_title('RMSE: Before vs After Compensation')
    ax.set_xlabel('Axis')
    ax.set_ylabel('RMSE (m/s²)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(axes_labels)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = 'vibration_quick_demo.png'
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\n✓ Demo plot saved as: {plot_file}")
    print("\nQuick demo completed successfully!")
    print("\nFor more comprehensive examples, run:")
    print("  Python: PYTHONPATH=PYTHON/src python PYTHON/src/vibration_examples.py")
    print("  MATLAB: run_vibration_examples")


if __name__ == "__main__":
    try:
        quick_demo()
    except Exception as e:
        print(f"\nDemo failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)